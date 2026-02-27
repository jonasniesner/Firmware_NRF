#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>

uint32_t timestamp(void);
void set_timestamp(uint32_t timestamp);
void sleep_mode_enter(void);
void app_feed_wdt(void);
void updatemsdata(void);
void get_msd_payload(uint8_t* out, uint8_t max_len, uint8_t* out_len);

// Get chip ID as hex string (last 3 bytes of DEVICEID[1], uppercase, 6 chars)
void getChipIdHex(char* buffer, uint8_t buffer_size);

// MSD data functions
void updatemsdata(void);
void get_msd_payload(uint8_t* out, uint8_t max_len, uint8_t* out_len);

// Reboot flag (extern for clearing on BLE connection)
extern uint8_t rebootFlag;

// Check if BLE is currently active (connected)
bool is_ble_active(void);

// Restart advertising with updated MSD data
void advertising_restart_with_updated_msd(void);

#endif
