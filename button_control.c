#include "button_control.h"
#include "EPD_driver.h"  // For digitalRead, pinMode macros
#include "constants.h"
#include "main.h"        // For timestamp(), updatemsdata(), is_ble_active(), advertising_restart_with_updated_msd()
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "structs.h"

// External reference to global config
extern struct GlobalConfig globalConfig;
extern uint8_t dynamicreturndata[11];  // Defined in main.c

// Button state tracking
static ButtonState buttonStates[MAX_BUTTONS] = {0};
static uint8_t buttonStateCount = 0;
static volatile bool buttonEventPending = false;
static volatile uint8_t lastChangedButtonIndex = BUTTON_INVALID_INDEX;

// Forward declaration
static void handle_button_isr(uint8_t buttonIndex);

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // Find which button this pin belongs to
    for (uint8_t i = 0; i < buttonStateCount; i++) {
        if (buttonStates[i].initialized && buttonStates[i].pin == pin) {
            handle_button_isr(i);
            break;
        }
    }
}

static void handle_button_isr(uint8_t buttonIndex) {
    if (buttonIndex >= MAX_BUTTONS || !buttonStates[buttonIndex].initialized) {
        return;
    }
    
    ButtonState* btn = &buttonStates[buttonIndex];
    bool pinState = digitalRead(btn->pin);
    bool pressed = btn->inverted ? !pinState : pinState;
    uint8_t newState = pressed ? 1 : 0;
    
    if (newState != btn->current_state) {
        btn->current_state = newState;
        lastChangedButtonIndex = buttonIndex;
        
        if (pressed) {
            if (btn->press_count < BUTTON_MAX_PRESS_COUNT) {
                btn->press_count++;
            }
        }
        
        buttonEventPending = true;
    }
}

void button_init(void) {
    // Initialize GPIOTE driver if not already initialized
    if (!nrf_drv_gpiote_is_init()) {
        ret_code_t err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("Failed to initialize GPIOTE: %d", err_code);
            return;
        }
    }
    
    buttonStateCount = 0;
    
    // Clear all button states
    for (uint8_t i = 0; i < MAX_BUTTONS; i++) {
        buttonStates[i].initialized = false;
        buttonStates[i].button_id = 0;
        buttonStates[i].press_count = 0;
        buttonStates[i].last_press_time = 0;
        buttonStates[i].current_state = 0;
        buttonStates[i].byte_index = BUTTON_INVALID_INDEX;
        buttonStates[i].pin = GPIO_PIN_UNUSED;
        buttonStates[i].instance_index = BUTTON_INVALID_INDEX;
    }
    
    if (globalConfig.binary_input_count == 0) {
        return;
    }
    
    // Scan BinaryInputs config for buttons (input_type == 1)
    for (uint8_t instanceIdx = 0; instanceIdx < globalConfig.binary_input_count; instanceIdx++) {
        struct BinaryInputs* input = &globalConfig.binary_inputs[instanceIdx];
        
        if (input->input_type != 1) {  // 1 = button type
            continue;
        }
        
        if (input->button_data_byte_index >= 11) {
            NRF_LOG_WARNING("BinaryInputs instance %d has invalid byte_index (%d), skipping",
                           instanceIdx, input->button_data_byte_index);
            continue;
        }
        
        // Array of pin pointers for this instance
        uint8_t* instancePins[8] = {
            &input->reserved_pin_1,
            &input->reserved_pin_2,
            &input->reserved_pin_3,
            &input->reserved_pin_4,
            &input->reserved_pin_5,
            &input->reserved_pin_6,
            &input->reserved_pin_7,
            &input->reserved_pin_8
        };
        
        // Process each pin in this instance
        for (uint8_t pinIdx = 0; pinIdx < 8; pinIdx++) {
            uint8_t pin = *instancePins[pinIdx];
            
            if (pin == GPIO_PIN_UNUSED) {
                continue;  // Pin not configured
            }
            
            if (buttonStateCount >= MAX_BUTTONS) {
                NRF_LOG_WARNING("Maximum button count (%d) reached, skipping remaining pins", MAX_BUTTONS);
                break;
            }
            
            ButtonState* btn = &buttonStates[buttonStateCount];
            
            // Calculate button ID (instance * 8 + pin offset, wrapped to 0-7)
            btn->button_id = (input->instance_number * 8) + pinIdx;
            if (btn->button_id > 7) {
                btn->button_id = btn->button_id % 8;
            }
            
            btn->byte_index = input->button_data_byte_index;
            btn->pin = pin;
            btn->instance_index = instanceIdx;
            btn->press_count = 0;
            btn->last_press_time = 0;
            btn->pin_offset = pinIdx;
            btn->inverted = (input->invert & (1 << pinIdx)) != 0;
            
            // Configure pin as input with pullup/pulldown
            bool hasPullup = (input->pullups & (1 << pinIdx)) != 0;
            bool hasPulldown = (input->pulldowns & (1 << pinIdx)) != 0;
            
            if (hasPullup) {
                pinMode(pin, INPUT_PULLUP);
            } else if (hasPulldown) {
                pinMode(pin, INPUT_PULLDOWN);
            } else {
                pinMode(pin, INPUT);
            }
            
            // Small delay to let pin settle
            nrf_delay_ms(BUTTON_PIN_SETTLE_MS);
            
            // Read initial state
            bool initialPinState = digitalRead(pin);
            bool initialPressed = btn->inverted ? !initialPinState : initialPinState;
            btn->current_state = initialPressed ? 1 : 0;
            
            // Configure GPIOTE interrupt for this pin
            nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
            if (hasPullup) {
                in_config.pull = NRF_GPIO_PIN_PULLUP;
            } else if (hasPulldown) {
                in_config.pull = NRF_GPIO_PIN_PULLDOWN;
            } else {
                in_config.pull = NRF_GPIO_PIN_NOPULL;
            }
            
            ret_code_t err_code = nrf_drv_gpiote_in_init(pin, &in_config, gpiote_event_handler);
            if (err_code != NRF_SUCCESS) {
                NRF_LOG_ERROR("Failed to initialize GPIOTE for pin %d: %d", pin, err_code);
                continue;
            }
            
            nrf_drv_gpiote_in_event_enable(pin, true);
            
            btn->initialized = true;
            buttonStateCount++;
        }
    }
}

void process_button_events(void) {
    if (!buttonEventPending) {
        return;
    }
    
    buttonEventPending = false;
    
    uint32_t currentTime = timestamp();  // Current time in seconds
    uint8_t changedButtonIndex = lastChangedButtonIndex;
    lastChangedButtonIndex = BUTTON_INVALID_INDEX;
    
    if (changedButtonIndex >= MAX_BUTTONS || !buttonStates[changedButtonIndex].initialized) {
        return;
    }
    
    ButtonState* btn = &buttonStates[changedButtonIndex];
    
    // Handle press count and timing logic
    if (btn->current_state == 1) {  // Button is pressed
        bool resetCount = false;
        
        // Reset count if first press or more than BUTTON_PRESS_TIMEOUT_SEC seconds since last press
        if (btn->last_press_time == 0 || (currentTime - btn->last_press_time) > BUTTON_PRESS_TIMEOUT_SEC) {
            resetCount = true;
        }
        
        // Reset count if another button was pressed more recently (within 5 seconds)
        if (btn->last_press_time > 0) {
            for (uint8_t j = 0; j < buttonStateCount; j++) {
                if (j != changedButtonIndex && buttonStates[j].initialized &&
                    buttonStates[j].last_press_time > 0 &&
                    buttonStates[j].last_press_time > btn->last_press_time &&
                    (currentTime - buttonStates[j].last_press_time) < BUTTON_PRESS_TIMEOUT_SEC) {
                    resetCount = true;
                    break;
                }
            }
        }
        
        if (resetCount) {
            // Reset all button press counts
            for (uint8_t j = 0; j < buttonStateCount; j++) {
                if (buttonStates[j].initialized) {
                    buttonStates[j].press_count = 0;
                }
            }
            btn->press_count = 1;
        }
        
        btn->last_press_time = currentTime;
    }
    
    // Read current pin state (may have changed since ISR)
    bool pinState = digitalRead(btn->pin);
    bool logicalPressed = btn->inverted ? !pinState : pinState;
    uint8_t logicalState = logicalPressed ? 1 : 0;
    btn->current_state = logicalState;
    
    // Encode button data: button_id (3 bits) | press_count (4 bits) | state (1 bit)
    uint8_t buttonData = (btn->button_id & BUTTON_ID_MASK) |                              // Bits 0-2: button_id
                         ((btn->press_count & PRESS_COUNT_MASK) << PRESS_COUNT_SHIFT) |   // Bits 3-6: press_count (0-15)
                         ((btn->current_state & BUTTON_STATE_MASK) << BUTTON_STATE_SHIFT); // Bit 7: current_state
    
    // Write to dynamicreturndata if byte_index is valid
    if (btn->byte_index < 11) {
        dynamicreturndata[btn->byte_index] = buttonData;
    }
    
    // Update MSD data with new button state
    updatemsdata();
    
    // Restart advertising to immediately reflect button state (only if not connected)
    if (!is_ble_active()) {
        advertising_restart_with_updated_msd();
    }
}
