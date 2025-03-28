/**
 * @file ld2450_parser.c
 * @brief Implementation of the HLK-LD2450 data frame parser
 * 
 * This file implements the parsing functions for the HLK-LD2450 radar data frames,
 * extracting target information and calculating derived metrics.
 * 
 * @author NieRVoid
 * @date 2025-03-15
 * @license MIT
 */

#include <string.h>
#include <math.h>
#include "ld2450.h"
#include "ld2450_private.h"
#include "ld2450_circular_buffer.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = LD2450_LOG_TAG;

/**
 * @brief Compress a raw speed value (int16_t) to a scaled int8_t representation
 * 
 * Maps the original speed value to int8_t range using the formula: 
 * (magnitude >> 3) + (magnitude >> 5) to approximate (magnitude * 128) / 1000
 * 
 * @param speed_raw Original speed value from radar
 * @return int8_t Compressed speed value
 */
static inline int8_t compress_speed(uint16_t speed_raw) {
    // Extract the 15-bit magnitude
    int16_t magnitude = speed_raw & 0x7FFF;
    
    // Apply the compression formula: ~= (magnitude * 128) / 1000
    int8_t compressed = (magnitude >> 3) + (magnitude >> 5);
    
    // Apply sign based on MSB of original value
    return (speed_raw & 0x8000) ? compressed : -compressed;
}

/**
 * @brief Parse target data from a data segment within a frame
 * 
 * @param target_data Pointer to target data segment (8 bytes)
 * @param target Pointer to target structure to fill
 * @return true if target is valid, false if target segment is empty
 */
static bool parse_target(const uint8_t *target_data, ld2450_target_t *target)
{
    // Check if target segment is empty (all zeros) using faster 32-bit checks
    uint32_t *data32 = (uint32_t*)target_data;
    if (data32[0] == 0 && data32[1] == 0) {
        return false;
    }
    
    // Parse target data according to protocol
    // X coordinate (little-endian)
    uint16_t x_raw = target_data[0] | (target_data[1] << 8);
    // Y coordinate (little-endian)
    uint16_t y_raw = target_data[2] | (target_data[3] << 8);
    // Speed (little-endian)
    uint16_t speed_raw = target_data[4] | (target_data[5] << 8);
    
    // Convert coordinates according to protocol
    // For X, Y, Speed: MSB 1 indicates positive, 0 indicates negative
    // Handle the 15-bit magnitude with proper sign
    int16_t x_magnitude = x_raw & 0x7FFF;  // Mask off MSB to get magnitude (15 bits)
    int16_t y_magnitude = y_raw & 0x7FFF;  // Mask off MSB to get magnitude (15 bits)
    
    // Apply sign based on MSB
    target->x = (target_data[1] & 0x80) ? x_magnitude : -x_magnitude;
    target->y = (target_data[3] & 0x80) ? y_magnitude : -y_magnitude;
    
    // Compress speed to int8_t using custom compression formula
    target->speed = compress_speed(speed_raw);
    
    return true;
}

/**
 * @brief Parse a complete data frame into a frame structure
 * 
 * @param data Raw frame data
 * @param len Length of the data
 * @param frame Frame structure to fill
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_parse_frame(const uint8_t *data, size_t len, ld2450_frame_t *frame)
{
    if (!data || !frame || len < LD2450_DATA_FRAME_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate frame header and footer
    if (memcmp(data, LD2450_DATA_FRAME_HEADER, 4) != 0 || 
        memcmp(data + len - 2, LD2450_DATA_FRAME_FOOTER, 2) != 0) {
        ESP_LOGW(TAG, "Invalid frame header or footer");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_DEBUG);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset valid mask
    frame->valid_mask = 0;
    
    // Parse up to 3 targets
    for (int i = 0; i < 3; i++) {
        // Target data starts at offset 4 and each target data segment is 8 bytes
        const uint8_t *target_data = data + 4 + (i * 8);
        
        // Parse target data
        if (parse_target(target_data, &frame->targets[i])) {
            // Set the corresponding bit in the valid mask
            frame->valid_mask |= (1 << i);
        } else {
            // Clear the target data for invalid targets
            memset(&frame->targets[i], 0, sizeof(ld2450_target_t));
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Handle a complete data frame
 * 
 * This function processes a complete data frame, parses it, and delivers
 * the results to the circular buffer for consumers.
 * 
 * @param data Frame data
 * @param len Frame length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_handle_data_frame(const uint8_t *data, size_t len)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Skip processing if we're in config mode
    if (instance->in_config_mode) {
        return ESP_OK;
    }
    
    // Static frame structure for parsing
    static ld2450_frame_t frame;
    
    // Parse the frame
    ret = ld2450_parse_frame(data, len, &frame);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write frame to circular buffer for multi-consumer access
    return ld2450_circular_buffer_write(&instance->circular_buffer, &frame);
}

/**
 * @brief Process radar data coming in from UART
 * 
 * This function processes incoming data from the UART, looking for valid frame
 * headers and footers, and passing complete frames to the parser.
 * 
 * @param byte Incoming byte to process
 * @return esp_err_t ESP_OK if a frame was successfully processed, ESP_ERR_NOT_FINISHED if still collecting data, or error code
 */
esp_err_t ld2450_process_byte(uint8_t byte)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Skip data processing if we're in config mode
    if (instance->in_config_mode) {
        return ESP_ERR_NOT_FINISHED;
    }
    
    // Check for frame header
    if (!instance->frame_synced) {
        // Shift bytes through a 4-byte window to look for header
        memmove(instance->frame_buffer, instance->frame_buffer + 1, 3);
        instance->frame_buffer[3] = byte;
        
        // Check if we've found a header
        if (memcmp(instance->frame_buffer, LD2450_DATA_FRAME_HEADER, 4) == 0) {
            instance->frame_synced = true;
            instance->frame_idx = 4; // We already have 4 bytes
            ESP_LOGV(TAG, "Frame sync acquired");
        }
        
        return ESP_ERR_NOT_FINISHED;
    }
    
    // If we're collecting a frame, add the byte
    if (instance->frame_idx < LD2450_DATA_FRAME_SIZE) {
        instance->frame_buffer[instance->frame_idx++] = byte;
        
        // Check if we have a complete frame
        if (instance->frame_idx == LD2450_DATA_FRAME_SIZE) {
            // Validate frame footer
            if (memcmp(instance->frame_buffer + LD2450_DATA_FRAME_SIZE - 2, 
                      LD2450_DATA_FRAME_FOOTER, 2) == 0) {
                
                // Process the complete frame
                esp_err_t ret = ld2450_handle_data_frame(instance->frame_buffer, LD2450_DATA_FRAME_SIZE);
                
                // Reset for next frame
                instance->frame_synced = false;
                instance->frame_idx = 0;
                
                return ret;
            } else {
                // Invalid footer, resync
                ESP_LOGW(TAG, "Invalid frame footer, resyncing");
                instance->frame_synced = false;
                instance->frame_idx = 0;
                
                return ESP_ERR_INVALID_STATE;
            }
        }
    }
    
    return ESP_ERR_NOT_FINISHED;
}

/**
 * @brief Process a chunk of UART data
 * 
 * @param data UART data buffer
 * @param len Length of data
 * @return esp_err_t ESP_OK if at least one frame was processed, otherwise error code
 */
esp_err_t ld2450_process_data(const uint8_t *data, size_t len)
{
    esp_err_t ret = ESP_ERR_NOT_FINISHED;
    esp_err_t last_result = ESP_ERR_NOT_FINISHED;
    
    // Process each byte
    for (size_t i = 0; i < len; i++) {
        last_result = ld2450_process_byte(data[i]);
        if (last_result == ESP_OK) {
            ret = ESP_OK; // At least one frame was successfully processed
        }
    }
    
    return ret;
}

/**
 * @brief UART event handler for batch processing of incoming data
 * 
 * @param data_buffer Buffer containing UART data
 * @param len Length of data in the buffer
 */
void ld2450_uart_event_handler(uint8_t *data_buffer, size_t len)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return;
    }
    
    // Skip data processing if we're in config mode
    if (instance->in_config_mode) {
        ESP_LOGV(TAG, "Skipping data processing while in config mode");
        return;
    }
    
    // Process multiple bytes at once using state machine approach
    for (int i = 0; i < len; i++) {
        if (!instance->frame_synced) {
            // Use efficient header detection with 4 consecutive bytes
            if (i <= len - 4) {
                if (memcmp(&data_buffer[i], LD2450_DATA_FRAME_HEADER, 4) == 0) {
                    instance->frame_synced = true;
                    memcpy(instance->frame_buffer, LD2450_DATA_FRAME_HEADER, 4);
                    instance->frame_idx = 4;
                    i += 3; // Skip the remaining header bytes
                }
            }
        } else if (instance->frame_idx < LD2450_DATA_FRAME_SIZE) {
            instance->frame_buffer[instance->frame_idx++] = data_buffer[i];
            
            // Process complete frame
            if (instance->frame_idx == LD2450_DATA_FRAME_SIZE) {
                // Validate footer
                if (memcmp(instance->frame_buffer + LD2450_DATA_FRAME_SIZE - 2, 
                           LD2450_DATA_FRAME_FOOTER, 2) == 0) {
                    ld2450_handle_data_frame(instance->frame_buffer, LD2450_DATA_FRAME_SIZE);
                }
                instance->frame_synced = false;
                instance->frame_idx = 0;
            }
        }
    }
}