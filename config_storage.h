#ifndef __CONFIG_STORAGE_H
#define __CONFIG_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_CONFIG_SIZE 512

// Config storage structure (matches src format)
typedef struct {
    uint32_t magic;        // 0xDEADBEEF
    uint32_t version;      // Config format version
    uint32_t crc;          // CRC32 of data
    uint32_t data_len;     // Actual data length
    uint8_t data[MAX_CONFIG_SIZE]; // Config data
} config_storage_t;

// Initialize config storage (FDS)
bool initConfigStorage(void);

// Save config data to flash
bool saveConfig(uint8_t* configData, uint32_t len);

// Load config data from flash
bool loadConfig(uint8_t* configData, uint32_t* len);

// Calculate CRC32 for config data
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);

#endif // __CONFIG_STORAGE_H
