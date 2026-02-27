#include "EPD_service.h"
#include <stdlib.h>
#include <string.h>
#include "app_scheduler.h"
#include "ble_srv_common.h"
#include "config_storage.h"
#include "constants.h"
#include "led_control.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"
#include "sdk_macros.h"

// Chunked write state for config write
typedef struct {
    bool active;
    uint32_t totalSize;
    uint16_t expectedChunks;
    uint16_t receivedChunks;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
} chunked_write_state_t;

static chunked_write_state_t chunkedWriteState = {0};

// Response buffer for config read
static uint8_t configReadResponseBuffer[MAX_RESPONSE_DATA_SIZE];

// Direct write state
typedef struct {
    bool active;
    bool bitplanes;          // True if using bitplanes (BWR/BWY - 2 planes)
    bool plane2;             // True when writing plane 2 (R/Y) for bitplanes
    uint32_t bytes_written;   // Total bytes written to current plane
    uint32_t total_bytes;     // Total bytes expected per plane (for bitplanes) or total (for others)
    uint8_t write_cfg;        // Current write configuration (plane/color)
} direct_write_state_t;

static direct_write_state_t directWriteState = {0};
static uint32_t direct_write_packet_count = 0; // Packet counter for logging (reset on start)

// Forward declarations
static uint32_t send_response(ble_epd_t* p_epd, uint8_t* response, uint16_t len);
static void handle_read_config(ble_epd_t* p_epd);
static void handle_write_config(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void handle_write_config_chunk(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void handle_direct_write_start(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void handle_direct_write_data(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void handle_direct_write_end(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void handle_led_activate(ble_epd_t* p_epd, uint8_t* data, uint16_t len);
static void cleanup_direct_write_state(ble_epd_t* p_epd);

// Helper functions for duplicate code patterns
static void send_error_response(ble_epd_t* p_epd, uint8_t error_code);
static void epd_sleep_and_uninit(ble_epd_t* p_epd);
static void invert_data_buffer(uint8_t* dest, const uint8_t* src, uint16_t len);
static void epd_reset_window(ble_epd_t* p_epd);

static void on_connect(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    if (p_epd->display_config != NULL && p_epd->epd != NULL) {
        NRF_LOG_INFO("EPD: %dx%d, type=0x%04X, model=%d, color=%d",
                     p_epd->display_config->pixel_width, p_epd->display_config->pixel_height,
                     p_epd->display_config->panel_ic_type, p_epd->epd->id, p_epd->epd->color);
    }
    
    p_epd->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_epd->conn_handle = BLE_CONN_HANDLE_INVALID;
    // If a direct-write session is active, ensure panel and GPIO are powered down.
    if (directWriteState.active && p_epd->epd && p_epd->epd->drv) {
        epd_sleep_and_uninit(p_epd);
        cleanup_direct_write_state(p_epd);
    }
}


static uint32_t send_response(ble_epd_t* p_epd, uint8_t* response, uint16_t len) {
    if (p_epd == NULL || response == NULL || len == 0) {
        return NRF_ERROR_NULL;
    }
    if (!p_epd->is_notification_enabled) {
        NRF_LOG_DEBUG("Notifications not enabled, cannot send response\n");
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_epd->conn_handle == BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_DEBUG("Not connected, cannot send response\n");
        return NRF_ERROR_INVALID_STATE;
    }
    return ble_epd_string_send(p_epd, response, len);
}

static void handle_read_config(ble_epd_t* p_epd) {
    // Use static buffer to avoid stack overflow
    static uint8_t configData[MAX_CONFIG_SIZE];
    uint32_t configLen = MAX_CONFIG_SIZE;
    
    // Ensure FDS is initialized
    if (!initConfigStorage()) {
        NRF_LOG_ERROR("Failed to initialize config storage\n");
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_READ, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    if (!loadConfig(configData, &configLen)) {
        // No config exists - return empty config (length 0) instead of error
        uint16_t responseLen = 0;
        configReadResponseBuffer[responseLen++] = 0x00; // Success
        configReadResponseBuffer[responseLen++] = RESP_CONFIG_READ; // Command echo
        configReadResponseBuffer[responseLen++] = 0x00; // Chunk 0, low byte
        configReadResponseBuffer[responseLen++] = 0x00; // Chunk 0, high byte
        configReadResponseBuffer[responseLen++] = 0x00; // Length 0, low byte
        configReadResponseBuffer[responseLen++] = 0x00; // Length 0, high byte
        (void)send_response(p_epd, configReadResponseBuffer, responseLen);
        return;
    }
    
    // Send config in chunks
    uint32_t remaining = configLen;
    uint32_t offset = 0;
    uint16_t chunkNumber = 0;
    const uint16_t maxChunks = 10;
    
    while (remaining > 0 && chunkNumber < maxChunks) {
        uint16_t responseLen = 0;
        configReadResponseBuffer[responseLen++] = 0x00; // Success
        configReadResponseBuffer[responseLen++] = RESP_CONFIG_READ; // Command echo
        configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
        configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
        
        if (chunkNumber == 0) {
            // First chunk includes total length
            configReadResponseBuffer[responseLen++] = configLen & 0xFF;
            configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
        }
        
        uint16_t maxDataSize = MAX_RESPONSE_DATA_SIZE - responseLen;
        uint16_t chunkSize = (remaining < maxDataSize) ? remaining : maxDataSize;
        
        if (chunkSize == 0) break;
        
        memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
        responseLen += chunkSize;
        
        if (responseLen > MAX_RESPONSE_DATA_SIZE) {
            break;
        }
        
        uint32_t err_code = send_response(p_epd, configReadResponseBuffer, responseLen);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("Failed to send config chunk %d: %d\n", chunkNumber, err_code);
            break;
        }
        
        offset += chunkSize;
        remaining -= chunkSize;
        chunkNumber++;
        
        // No delay - BLE stack handles flow control automatically
        // Blocking delays in BLE handlers can cause SoftDevice timeouts
    }
    
}

static void handle_write_config(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    if (len == 0) {
        return;
    }
    
    if (len > CONFIG_CHUNK_SIZE) {
        // Start chunked write
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        
        if (len >= CONFIG_CHUNK_SIZE_WITH_PREFIX) {
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + CONFIG_CHUNK_SIZE - 1) / CONFIG_CHUNK_SIZE;
            uint16_t chunkDataSize = ((len - 2) < CONFIG_CHUNK_SIZE) ? (len - 2) : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
        } else {
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            uint16_t chunkSize = (len < CONFIG_CHUNK_SIZE) ? len : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
        }
        
        uint8_t ackResponse[] = {0x00, RESP_CONFIG_WRITE, 0x00, 0x00};
        (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
        return;
    }
    
    // Single write
    if (saveConfig(data, len)) {
        uint8_t successResponse[] = {0x00, RESP_CONFIG_WRITE, 0x00, 0x00};
        (void)send_response(p_epd, successResponse, sizeof(successResponse));
    } else {
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_WRITE, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
    }
}

static void handle_firmware_version(ble_epd_t* p_epd) {
    // Parse BUILD_VERSION string (e.g., "1.0") to extract major and minor
    // Default to APP_VERSION if BUILD_VERSION parsing fails
    uint8_t major = 0;
    uint8_t minor = 0;
    
    // Try to parse BUILD_VERSION string
    const char* build_ver = BUILD_VERSION;
    if (build_ver != NULL && strlen(build_ver) > 0) {
        // Simple parsing: look for "X.Y" format
        major = (uint8_t)atoi(build_ver);
        const char* dot = strchr(build_ver, '.');
        if (dot != NULL && dot[1] != '\0') {
            minor = (uint8_t)atoi(dot + 1);
        }
    }
    
    // If parsing failed, use APP_VERSION (0x19 = 25)
    if (major == 0 && minor == 0) {
        major = (APP_VERSION >> 8) & 0xFF;
        minor = APP_VERSION & 0xFF;
    }
    
    // Get SHA string - SHA_STRING is defined as a string literal by build system
    const char* sha_str = SHA_STRING;
    uint8_t sha_len = 0;
    
    // Calculate length (SHA_STRING is already a string literal, no quotes to remove)
    if (sha_str != NULL) {
        sha_len = strlen(sha_str);
        // Limit to 40 characters
        if (sha_len > 40) sha_len = 40;
    }
    
    // Build response: [0x00, 0x43, major, minor, sha_len, ...sha_bytes...]
    uint8_t response[2 + 1 + 1 + 1 + 40];
    uint16_t offset = 0;
    response[offset++] = 0x00;  // Success
    response[offset++] = RESP_FIRMWARE_VERSION;  // Command echo
    response[offset++] = major;
    response[offset++] = minor;
    response[offset++] = sha_len;
    
    // Copy SHA bytes
    if (sha_str != NULL && sha_len > 0) {
        for (uint8_t i = 0; i < sha_len && i < 40; i++) {
            response[offset++] = sha_str[i];
        }
    }
    
    (void)send_response(p_epd, response, offset);
}

static void handle_write_config_chunk(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    if (!chunkedWriteState.active) {
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    if (len == 0) {
        return;
    }
    
    if (len > CONFIG_CHUNK_SIZE) {
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    if (chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE) {
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    if (chunkedWriteState.receivedChunks >= MAX_CONFIG_CHUNKS) {
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        // All chunks received, save config
        if (saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize)) {
            uint8_t successResponse[] = {0x00, RESP_CONFIG_CHUNK, 0x00, 0x00};
            (void)send_response(p_epd, successResponse, sizeof(successResponse));
        } else {
            uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
            (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        }
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        // Acknowledge chunk
        uint8_t ackResponse[] = {0x00, RESP_CONFIG_CHUNK, 0x00, 0x00};
        (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
    }
}

// Helper function: Send error response
static void send_error_response(ble_epd_t* p_epd, uint8_t error_code) {
    uint8_t errorResponse[] = {0xFF, error_code};
    (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
}

// Helper function: Put EPD to sleep and uninitialize GPIO
static void epd_sleep_and_uninit(ble_epd_t* p_epd) {
    if (p_epd->epd && p_epd->epd->drv && p_epd->epd->drv->sleep) {
    p_epd->epd->drv->sleep(p_epd->epd);
        nrf_delay_ms(EPD_SLEEP_DELAY_MS);
    }
    EPD_GPIO_Uninit();
}

// Helper function: Invert data buffer for plane 2
static void invert_data_buffer(uint8_t* dest, const uint8_t* src, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        dest[i] = ~src[i];
    }
}

// Helper function: Reset EPD window to (0,0) with full size
static void epd_reset_window(ble_epd_t* p_epd) {
    if (p_epd->epd && p_epd->epd->drv && p_epd->epd->drv->set_window) {
        p_epd->epd->drv->set_window(p_epd->epd, 0, 0, 
                                     p_epd->epd->width, p_epd->epd->height);
    }
}

static void cleanup_direct_write_state(ble_epd_t* p_epd) {
    directWriteState.active = false;
    directWriteState.bitplanes = false;
    directWriteState.plane2 = false;
    directWriteState.bytes_written = 0;
    directWriteState.total_bytes = 0;
    directWriteState.write_cfg = 0;
    direct_write_packet_count = 0;
}

static void handle_direct_write_start(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    if (p_epd->epd == NULL) {
        NRF_LOG_ERROR("DW: EPD not initialized\n");
        send_error_response(p_epd, RESP_DIRECT_WRITE_ERROR);
        return;
    }
    
    // Cleanup any existing direct write session
    if (directWriteState.active) {
        cleanup_direct_write_state(p_epd);
    }
    
    // Ensure GPIO is initialized
    EPD_GPIO_Init();
    
    // Re-initialize display to ensure clean state for new write
    if (p_epd->epd->drv && p_epd->epd->drv->init) {
        p_epd->epd->drv->init(p_epd->epd);
    }
    
    // Determine if using bitplanes based on display color scheme
    // BWR/BWY displays (color == COLOR_BWR) use bitplanes
    directWriteState.bitplanes = (p_epd->epd->color == COLOR_BWR);
    directWriteState.plane2 = false;
    
    // Calculate expected total bytes per plane.
    // Use row stride (byte-aligned rows), not packed pixel math, so widths that are
    // not multiples of 8 (e.g. 300 px) do not drift and corrupt bottom rows.
    uint32_t row_bytes = ((uint32_t)p_epd->epd->width + 7U) / 8U;
    directWriteState.total_bytes = row_bytes * (uint32_t)p_epd->epd->height;
    
    // Start write_ram with begin flag for plane 1 (black)
    // EPD_WRITE_CFG_PLANE1_BEGIN means: begin=true (upper nibble 0x0), black=true (lower nibble 0xF)
    // SSD16xx_WriteRam will set the window automatically when begin=true && black=true
    // So we don't need to set it explicitly here
    directWriteState.write_cfg = EPD_WRITE_CFG_PLANE1_BEGIN;
    directWriteState.active = true;
    directWriteState.bytes_written = 0;
    direct_write_packet_count = 0;
    
    // Send acknowledgment
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_START_ACK};
    (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
}

static void handle_direct_write_data(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    if (!directWriteState.active) {
        NRF_LOG_ERROR("DW: Data received but not active\n");
        send_error_response(p_epd, RESP_DIRECT_WRITE_ERROR);
        return;
    }
    
    if (p_epd->epd == NULL || p_epd->epd->drv == NULL || p_epd->epd->drv->write_ram == NULL) {
        NRF_LOG_ERROR("DW: Driver not available\n");
        cleanup_direct_write_state(p_epd);
        send_error_response(p_epd, RESP_DIRECT_WRITE_ERROR);
        return;
    }
    
    if (len == 0) {
        uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_DATA_ACK};
        (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
        return;
    }
    
    direct_write_packet_count++;
    uint16_t stream_offset = 0;

    // Consume incoming bytes as a continuous stream across plane boundaries.
    while (stream_offset < len) {
        // Switch to plane 2 exactly when plane 1 is complete.
        if (directWriteState.bitplanes &&
            !directWriteState.plane2 &&
            directWriteState.bytes_written >= directWriteState.total_bytes) {
            epd_reset_window(p_epd);
            directWriteState.plane2 = true;
            directWriteState.bytes_written = 0;
            directWriteState.write_cfg = EPD_WRITE_CFG_PLANE2_BEGIN;
        }

        // If current plane is already full, remaining bytes are true overrun.
        uint32_t remaining_bytes = directWriteState.total_bytes - directWriteState.bytes_written;
        if (remaining_bytes == 0) {
            NRF_LOG_WARNING("DW: Overrun! Received %d extra bytes after plane completion\n", (len - stream_offset));
            break;
        }

        uint16_t bytes_to_write = len - stream_offset;
        if (bytes_to_write > remaining_bytes) {
            bytes_to_write = (uint16_t)remaining_bytes;
        }

        // First chunk of a plane uses BEGIN cfg, then CONTINUE.
        uint8_t base_cfg = directWriteState.plane2 ? EPD_WRITE_CFG_PLANE2_CONTINUE : EPD_WRITE_CFG_PLANE1_CONTINUE;
        if (directWriteState.bytes_written == 0) {
            base_cfg = directWriteState.write_cfg;
            if (directWriteState.plane2) {
                epd_reset_window(p_epd);
            }
        }

        uint16_t local_offset = 0;
        while (local_offset < bytes_to_write) {
            uint16_t chunk_size = bytes_to_write - local_offset;
            if (chunk_size > EPD_SPI_CHUNK_SIZE) {
                chunk_size = EPD_SPI_CHUNK_SIZE;
            }

            uint8_t chunk_cfg = (local_offset == 0) ? base_cfg :
                (directWriteState.plane2 ? EPD_WRITE_CFG_PLANE2_CONTINUE : EPD_WRITE_CFG_PLANE1_CONTINUE);
            uint8_t* chunk_data = data + stream_offset + local_offset;

            // UC8151 and UC 4.3 variants expect plane 2 as-is.
            // SSD16xx needs firmware inversion: web sends RED plane as 1=red/0=no-red,
            // but the controller with DISP_CTRL1 RED inverse expects 0=red/1=no-red.
            bool should_invert_plane2 = directWriteState.plane2 && 
                                        p_epd->epd->ic != DRV_IC_UC8151 &&
                                        p_epd->epd->ic != DRV_IC_UCVAR43;
            bool should_invert_plane1_013 = !directWriteState.plane2 &&
                                            (p_epd->epd->id == SSD1619_013_BW ||
                                             p_epd->epd->id == SSD1619_013_BWR);

            if (should_invert_plane2 || should_invert_plane1_013) {
                // Plane-2 inversion is required for SSD16xx red polarity.
                // 1.3" SSD1619 also needs plane-1 inversion to match web BW polarity.
                uint8_t inverted_chunk[EPD_SPI_CHUNK_SIZE];
                invert_data_buffer(inverted_chunk, chunk_data, chunk_size);
                p_epd->epd->drv->write_ram(p_epd->epd, chunk_cfg, inverted_chunk, chunk_size);
            } else {
                p_epd->epd->drv->write_ram(p_epd->epd, chunk_cfg, chunk_data, chunk_size);
            }

            local_offset += chunk_size;
            directWriteState.bytes_written += chunk_size;
        }

        stream_offset += bytes_to_write;
    }
    
    // Send acknowledgment
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_DATA_ACK};
    (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
}

static void handle_direct_write_end(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    if (!directWriteState.active) {
        NRF_LOG_WARNING("DW: End called but not active\n");
        return;
    }
    
    if (p_epd->epd == NULL || p_epd->epd->drv == NULL) {
        NRF_LOG_ERROR("EPD driver not available\n");
        cleanup_direct_write_state(p_epd);
        send_error_response(p_epd, RESP_DIRECT_WRITE_ERROR);
        EPD_GPIO_Uninit();
        return;
    }
    
    // Send end acknowledgment
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_END_ACK};
    (void)send_response(p_epd, ackResponse, sizeof(ackResponse));
    
    // For UC81xx displays, send PTOUT after completing all writes
    if (p_epd->epd->ic == DRV_IC_UC8176 || p_epd->epd->ic == DRV_IC_UC8179) {
        EPD_WriteCmd(UC81xx_PTOUT);
            }
    
    // Refresh display
    if (p_epd->epd->drv->refresh) {
        p_epd->epd->drv->refresh(p_epd->epd);
        
        // Wait for refresh to complete (with timeout)
        uint32_t timeout_ms = EPD_REFRESH_TIMEOUT_MS;
        uint32_t elapsed_ms = 0;
        bool refresh_success = false;
        
        while (elapsed_ms < timeout_ms) {
            if (p_epd->epd->drv->read_busy && !p_epd->epd->drv->read_busy(p_epd->epd)) {
                refresh_success = true;
            break;
            }
            nrf_delay_ms(EPD_REFRESH_CHECK_INTERVAL_MS);
            elapsed_ms += EPD_REFRESH_CHECK_INTERVAL_MS;
        }
        
        cleanup_direct_write_state(p_epd);
        epd_sleep_and_uninit(p_epd);
        
        // Send refresh result
        if (refresh_success) {
            uint8_t refreshResponse[] = {0x00, RESP_DIRECT_WRITE_REFRESH_SUCCESS};
            (void)send_response(p_epd, refreshResponse, sizeof(refreshResponse));
        } else {
            uint8_t timeoutResponse[] = {0x00, RESP_DIRECT_WRITE_REFRESH_TIMEOUT};
            (void)send_response(p_epd, timeoutResponse, sizeof(timeoutResponse));
            NRF_LOG_WARNING("DW: Display refresh timed out\n");
        }
    } else {
        cleanup_direct_write_state(p_epd);
        epd_sleep_and_uninit(p_epd);
        send_error_response(p_epd, RESP_DIRECT_WRITE_ERROR);
        NRF_LOG_ERROR("Display refresh function not available\n");
    }
}

static void handle_led_activate(ble_epd_t* p_epd, uint8_t* data, uint16_t len) {
    NRF_LOG_INFO("=== LED ACTIVATE COMMAND (0x0073) ===\n");
    
    // LED activation command format:
    // data[0]: LED instance number (0-based index)
    // data[1-12]: LED flash config (12 bytes, same format as reserved[0-11] in LedConfig)
    // If len == 1, only instance is provided (use existing config for that instance)
    // If len == 13, both instance and config are provided
    
    if (len < 1) {
        NRF_LOG_ERROR("LED activate command too short (len=%d, need at least 1 byte)\n", len);
        uint8_t errorResponse[] = {0xFF, RESP_LED_ACTIVATE_ACK, 0x01, 0x00};  // Error, command, error code, no data
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    uint8_t ledInstance = data[0];
    NRF_LOG_INFO("LED instance: %d\n", ledInstance);
    
    if (p_epd->global_config == NULL || ledInstance >= p_epd->global_config->led_count) {
        NRF_LOG_ERROR("LED instance %d out of range (led_count=%d)\n", 
                     ledInstance, 
                     p_epd->global_config ? p_epd->global_config->led_count : 0);
        uint8_t errorResponse[] = {0xFF, RESP_LED_ACTIVATE_ACK, 0x02, 0x00};  // Error, command, error code, no data
        (void)send_response(p_epd, errorResponse, sizeof(errorResponse));
        return;
    }
    
    struct LedConfig* led = &p_epd->global_config->leds[ledInstance];
    led_activate(ledInstance);
    
    NRF_LOG_INFO("Using LED instance %d: type=%d R=%d G=%d B=%d\n",
                 ledInstance, led->led_type, led->led_1_r, led->led_2_g, led->led_3_b);
    
    uint8_t* ledcfg = led->reserved;
    
    if (len >= 13) {
        NRF_LOG_INFO("Updating LED flash config from command data\n");
        memcpy(ledcfg, data + 1, 12);
        uint8_t modeByte = ledcfg[0];
        uint8_t mode = modeByte & 0x0F;
        uint8_t brightnessRaw = (modeByte >> 4) & 0x0F;
        uint8_t brightness = brightnessRaw + 1;
        NRF_LOG_INFO("Mode byte 0x%02X: mode=%d brightness=%d\n", modeByte, mode, brightness);
        if (mode == 0) {
            NRF_LOG_WARNING("Mode is 0 (disabled)! Set bit 0 to 1 for mode 1.\n");
        }
    } else {
        NRF_LOG_INFO("No config provided (len=%d), using existing config\n", len);
    }
    
    NRF_LOG_INFO("Activating LED flash\n");
    led_set_flash_active(true);
    ledFlashLogic();
    led_set_flash_active(false);
    
    uint8_t successResponse[] = {0x00, RESP_LED_ACTIVATE_ACK, 0x00, 0x00};  // Success, command, no error, no data
    (void)send_response(p_epd, successResponse, sizeof(successResponse));
    NRF_LOG_INFO("LED flash completed\n");
}

static void epd_service_on_write(ble_epd_t* p_epd, uint8_t* p_data, uint16_t length) {
    // Raw packet logging disabled for direct write debugging
    // NRF_LOG_DEBUG("[EPD]: on_write LEN=%d\n", length);
    // NRF_LOG_HEXDUMP_DEBUG(p_data, length);
    if (p_data == NULL || length <= 0) return;

    // Parse 16-bit command format (big-endian: data[0] << 8 | data[1])
    if (length < 2) {
        NRF_LOG_DEBUG("Command too short: %d bytes\n", length);
        return;
    }
    
    uint16_t command_16bit = (p_data[0] << 8) | p_data[1];
    
    // Handle commands
    switch (command_16bit) {
        case CMD_CONFIG_READ:
            handle_read_config(p_epd);
            break;
        case CMD_CONFIG_WRITE:
            handle_write_config(p_epd, &p_data[2], length - 2);
            break;
        case CMD_CONFIG_CHUNK:
            handle_write_config_chunk(p_epd, &p_data[2], length - 2);
            break;

        case CMD_DIRECT_WRITE_START:
            handle_direct_write_start(p_epd, &p_data[2], length - 2);
            break;

        case CMD_DIRECT_WRITE_DATA:
            handle_direct_write_data(p_epd, &p_data[2], length - 2);
            break;

        case CMD_DIRECT_WRITE_END:
            handle_direct_write_end(p_epd, &p_data[2], length - 2);
            break;

        case CMD_LED_ACTIVATE:
            handle_led_activate(p_epd, &p_data[2], length - 2);
            break;

        case CMD_REBOOT:
            NRF_LOG_INFO("Reboot command received (0x000F)\n");
            nrf_delay_ms(100);
            // System reset
            NVIC_SystemReset();
            break;

        case CMD_FIRMWARE_VERSION:
            handle_firmware_version(p_epd);
            break;

        default:
            NRF_LOG_DEBUG("Unknown command: 0x%04X\n", command_16bit);
            break;
    }
}

static void on_write(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_epd->char_handles.cccd_handle) && (p_evt_write->len == 2)) {
        if (ble_srv_is_notification_enabled(p_evt_write->data)) {
            NRF_LOG_DEBUG("notification enabled\n");
            p_epd->is_notification_enabled = true;
            // Config read now handled by config command handler
        } else {
            p_epd->is_notification_enabled = false;
        }
    } else if (p_evt_write->handle == p_epd->char_handles.value_handle) {
        epd_service_on_write(p_epd, p_evt_write->data, p_evt_write->len);
    } else {
        // Do Nothing. This event is not relevant for this service.
    }
}

#if defined(S112)
void ble_epd_evt_handler(ble_evt_t const* p_ble_evt, void* p_context) {
    if (p_context == NULL || p_ble_evt == NULL) return;

    ble_epd_t* p_epd = (ble_epd_t*)p_context;
    ble_epd_on_ble_evt(p_epd, (ble_evt_t*)p_ble_evt);
}
#endif

void ble_epd_on_ble_evt(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    if ((p_epd == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_epd, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_epd, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_epd, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t epd_service_init(ble_epd_t* p_epd) {
    ble_uuid_t ble_uuid = {0};
    ble_uuid128_t base_uuid = BLE_UUID_EPD_SVC_BASE;
    ble_add_char_params_t add_char_params;
    uint16_t app_version = APP_VERSION;
    uint8_t app_version_bytes[2] = {(uint8_t)(app_version & 0xFF), (uint8_t)((app_version >> 8) & 0xFF)};

    VERIFY_SUCCESS(sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type));

    ble_uuid.type = ble_uuid.type;
    ble_uuid.uuid = BLE_UUID_EPD_SVC;
    VERIFY_SUCCESS(sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_epd->service_handle));
    
    // Initialize config storage
    initConfigStorage();

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_EPD_CHAR;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = BLE_EPD_MAX_DATA_LEN;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.is_var_len = true;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.read_access = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    VERIFY_SUCCESS(characteristic_add(p_epd->service_handle, &add_char_params, &p_epd->char_handles));

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_APP_VER;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(uint16_t);
    add_char_params.init_len = sizeof(uint16_t);
    add_char_params.p_init_value = app_version_bytes;
    add_char_params.char_props.read = 1;
    add_char_params.read_access = SEC_OPEN;

    return characteristic_add(p_epd->service_handle, &add_char_params, &p_epd->app_ver_handles);
}

void ble_epd_sleep_prepare(ble_epd_t* p_epd) {
    // Turn off led
    // LED control is now handled by led_control.c
    // Wakeup pin handling removed - not in DisplayConfig
}

uint32_t ble_epd_init(ble_epd_t* p_epd) {
    if (p_epd == NULL) return NRF_ERROR_NULL;

    // Save display_config before memset (if it was set)
    struct DisplayConfig* saved_display_config = p_epd->display_config;

    // Initialize the service structure.
    p_epd->max_data_len = BLE_EPD_MAX_DATA_LEN;
    p_epd->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_epd->is_notification_enabled = false;

    // Restore display_config after memset so it can be used
    p_epd->display_config = saved_display_config;

    // Load GPIO from DisplayConfig if available
    if (p_epd->display_config != NULL) {
        EPD_GPIO_Load_DisplayConfig(p_epd->display_config, p_epd->global_config);
    }

    // LED control is now handled by led_control.c

    // Add the service.
    return epd_service_init(p_epd);
}

uint32_t ble_epd_string_send(ble_epd_t* p_epd, uint8_t* p_string, uint16_t length) {
    if ((p_epd->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_epd->is_notification_enabled))
        return NRF_ERROR_INVALID_STATE;
    if (length > p_epd->max_data_len) return NRF_ERROR_INVALID_PARAM;

    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_epd->char_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_epd->conn_handle, &hvx_params);
}

void ble_epd_on_timer(ble_epd_t* p_epd, uint32_t timestamp, bool force_update) {
    // MODE_PICTURE: No automatic updates needed, images are uploaded via BLE
    (void)p_epd;
    (void)timestamp;
    (void)force_update;
}
