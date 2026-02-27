#include "config_parser.h"
#include "constants.h"
#include "config_storage.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include <string.h>

#define TRANSMISSION_MODE_CLEAR_ON_BOOT (1 << 7)

bool parseConfigBytes(uint8_t* configData, uint32_t configLen, struct GlobalConfig* globalConfig) {
    if (globalConfig == NULL || configData == NULL) {
        NRF_LOG_ERROR("Invalid parameters for parseConfigBytes\n");
        return false;
    }
    
    // Initialize global config
    memset(globalConfig, 0, sizeof(struct GlobalConfig));
    
    if (configLen < 3) {
        NRF_LOG_ERROR("Config too short: %d bytes\n", configLen);
        globalConfig->loaded = false;
        return false;
    }
    
    NRF_LOG_INFO("Parsing config: %d bytes", configLen);
    
    uint32_t offset = 0;
    // Skip length field (2 bytes)
    offset += 2;
    
    // Read version
    globalConfig->version = configData[offset++];
    globalConfig->minor_version = 0; // Not stored in current format
    
    // Parse packets
    uint32_t packetIndex = 0;
    while (offset < configLen - 2) { // -2 for CRC
        uint32_t remaining = configLen - 2 - offset;
        if (offset + 2 > configLen - 2) {
            NRF_LOG_DEBUG("Loop exit: not enough for header (need 2, have %d)", remaining);
            break;
        }
        
        uint8_t packetNum = configData[offset];
        uint8_t packetId = configData[offset + 1];
        offset += 2; // Advance past packet header
        packetIndex++; // Count this packet (before processing, so we count even if we skip it)
        // Only log relevant packets
        if (packetId == CONFIG_PKT_SYSTEM || packetId == CONFIG_PKT_MANUFACTURER || 
            packetId == CONFIG_PKT_POWER || packetId == CONFIG_PKT_DISPLAY) {
            NRF_LOG_INFO("Pkt #%d ID=0x%02X", packetNum, packetId);
        }
        
        switch (packetId) {
            case CONFIG_PKT_SYSTEM: // system_config
                if (offset + sizeof(struct SystemConfig) <= configLen - 2) {
                    memcpy(&globalConfig->system_config, &configData[offset], sizeof(struct SystemConfig));
                    offset += sizeof(struct SystemConfig);
                } else {
                    NRF_LOG_ERROR("system_config: need %d, have %d", sizeof(struct SystemConfig), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_MANUFACTURER: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig->manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                } else {
                    NRF_LOG_ERROR("manufacturer_data: need %d, have %d", sizeof(struct ManufacturerData), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_POWER: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig->power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                } else {
                    NRF_LOG_ERROR("power_option: need %d, have %d", sizeof(struct PowerOption), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_DISPLAY: // display
                if (globalConfig->display_count < 4 && offset + sizeof(struct DisplayConfig) <= configLen - 2) {
                    memcpy(&globalConfig->displays[globalConfig->display_count], &configData[offset], sizeof(struct DisplayConfig));
                    // Log only relevant display fields (skip reserved/unused pins)
                    NRF_LOG_INFO("Display: ic=0x%04X %dx%d", 
                                 globalConfig->displays[globalConfig->display_count].panel_ic_type,
                                 globalConfig->displays[globalConfig->display_count].pixel_width,
                                 globalConfig->displays[globalConfig->display_count].pixel_height);
                    NRF_LOG_INFO("Display: RST=%d BUSY=%d DC=%d", 
                                 globalConfig->displays[globalConfig->display_count].reset_pin,
                                 globalConfig->displays[globalConfig->display_count].busy_pin,
                                 globalConfig->displays[globalConfig->display_count].dc_pin);
                    NRF_LOG_INFO("Display: CS=%d DATA=%d CLK=%d", 
                                 globalConfig->displays[globalConfig->display_count].cs_pin,
                                 globalConfig->displays[globalConfig->display_count].data_pin,
                                 globalConfig->displays[globalConfig->display_count].clk_pin);
                    NRF_LOG_INFO("Display: color=%d modes=0x%02X", 
                                 globalConfig->displays[globalConfig->display_count].color_scheme,
                                 globalConfig->displays[globalConfig->display_count].transmission_modes);
                    offset += sizeof(struct DisplayConfig);
                    globalConfig->display_count++;
                } else if (globalConfig->display_count >= 4) {
                    offset += sizeof(struct DisplayConfig);
                } else {
                    NRF_LOG_ERROR("display: need %d, have %d", sizeof(struct DisplayConfig), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_LED: // led - parse but don't log
                if (globalConfig->led_count < 4 && offset + sizeof(struct LedConfig) <= configLen - 2) {
                    memcpy(&globalConfig->leds[globalConfig->led_count], &configData[offset], sizeof(struct LedConfig));
                    offset += sizeof(struct LedConfig);
                    globalConfig->led_count++;
                } else if (globalConfig->led_count >= 4) {
                    offset += sizeof(struct LedConfig);
                } else {
                    NRF_LOG_ERROR("led: need %d, have %d", sizeof(struct LedConfig), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_SENSOR: // sensor_data - parse but don't log
                if (globalConfig->sensor_count < 4 && offset + sizeof(struct SensorData) <= configLen - 2) {
                    memcpy(&globalConfig->sensors[globalConfig->sensor_count], &configData[offset], sizeof(struct SensorData));
                    offset += sizeof(struct SensorData);
                    globalConfig->sensor_count++;
                } else if (globalConfig->sensor_count >= 4) {
                    offset += sizeof(struct SensorData);
                } else {
                    NRF_LOG_ERROR("sensor: need %d, have %d", sizeof(struct SensorData), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_DATA_BUS: // data_bus - parse but don't log
                if (globalConfig->data_bus_count < 4 && offset + sizeof(struct DataBus) <= configLen - 2) {
                    memcpy(&globalConfig->data_buses[globalConfig->data_bus_count], &configData[offset], sizeof(struct DataBus));
                    offset += sizeof(struct DataBus);
                    globalConfig->data_bus_count++;
                } else if (globalConfig->data_bus_count >= 4) {
                    offset += sizeof(struct DataBus);
                } else {
                    NRF_LOG_ERROR("data_bus: need %d, have %d", sizeof(struct DataBus), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_BINARY_INPUT: // binary_inputs - parse but don't log
                if (globalConfig->binary_input_count < 4 && offset + sizeof(struct BinaryInputs) <= configLen - 2) {
                    memcpy(&globalConfig->binary_inputs[globalConfig->binary_input_count], &configData[offset], sizeof(struct BinaryInputs));
                    offset += sizeof(struct BinaryInputs);
                    globalConfig->binary_input_count++;
                } else if (globalConfig->binary_input_count >= 4) {
                    offset += sizeof(struct BinaryInputs);
                } else {
                    NRF_LOG_ERROR("binary_input: need %d, have %d", sizeof(struct BinaryInputs), configLen - 2 - offset);
                    globalConfig->loaded = false;
                    return false;
                }
                break;
                
            case CONFIG_PKT_WIFI: // wifi_config - skip this as requested
                // Skip 162 bytes for wifi_config
                if (offset + 162 <= configLen - 2) {
                    offset += 162;
                } else {
                    offset = configLen - 2; // Skip to CRC
                }
                break;
                
            default:
                NRF_LOG_WARNING("Unknown pkt 0x%02X @%d", packetId, offset - 2);
                // Skip to CRC on unknown packet (like src does)
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    
    NRF_LOG_INFO("Parsed %d pkts, offset=%d/%d", packetIndex, offset, configLen - 2);
    
    // Verify CRC if we have enough data
    if (configLen >= 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        uint32_t crcCalculated32 = calculateConfigCRC(configData, configLen - 2);
        uint16_t crcCalculated = (uint16_t)(crcCalculated32 & 0xFFFF);
        if (crcGiven != crcCalculated) {
            NRF_LOG_WARNING("CRC mismatch: 0x%04X vs 0x%04X", crcGiven, crcCalculated);
        }
    }
    
    globalConfig->loaded = true;
    NRF_LOG_INFO("Config parsed successfully: version=%d, displays=%d, leds=%d, sensors=%d, data_buses=%d, binary_inputs=%d",
                 globalConfig->version, globalConfig->display_count, globalConfig->led_count,
                 globalConfig->sensor_count, globalConfig->data_bus_count, globalConfig->binary_input_count);
    return true;
}

bool loadGlobalConfig(struct GlobalConfig* globalConfig) {
    if (globalConfig == NULL) {
        NRF_LOG_ERROR("Invalid parameter for loadGlobalConfig\n");
        return false;
    }
    
    // Initialize global config to zeros (defaults)
    memset(globalConfig, 0, sizeof(struct GlobalConfig));
    globalConfig->loaded = false;
    
    static uint8_t configData[MAX_CONFIG_SIZE];
    uint32_t configLen = MAX_CONFIG_SIZE;
    
    if (!initConfigStorage()) {
        NRF_LOG_ERROR("Failed to initialize config storage\n");
        return false;
    }
    
    if (!loadConfig(configData, &configLen)) {
        NRF_LOG_DEBUG("No config found\n");
        // Config already initialized to zeros above
        return false;
    }
    
    return parseConfigBytes(configData, configLen, globalConfig);
}
