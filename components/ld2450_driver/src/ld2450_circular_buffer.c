/**
 * @file ld2450_circular_buffer.c
 * @brief Implementation of circular buffer for LD2450 radar frames
 * 
 * @author NieRVoid
 * @date 2025-03-15
 * @license MIT
 */

#include <string.h>
#include "ld2450_circular_buffer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

static const char *TAG = "LD2450_BUFFER";

/**
 * @brief Calculate buffer index with wrap-around using bit masking
 * Buffer size must be power of 2 for this to work properly
 */
static inline uint32_t get_index(uint32_t index) {
    return index & (LD2450_CIRCULAR_BUFFER_SIZE - 1);
}

esp_err_t ld2450_circular_buffer_init(ld2450_circular_buffer_t *buffer) {
    if (buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear entire buffer structure
    memset(buffer, 0, sizeof(ld2450_circular_buffer_t));
    
    // Verify buffer size is power of 2 at compile time
    _Static_assert((LD2450_CIRCULAR_BUFFER_SIZE & (LD2450_CIRCULAR_BUFFER_SIZE - 1)) == 0, 
                  "Buffer size must be power of 2");
    
    // Create mutex for consumer registration
    buffer->consumer_mutex = xSemaphoreCreateMutex();
    if (buffer->consumer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create consumer mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Circular buffer initialized with %d frames (%d bytes)", 
             LD2450_CIRCULAR_BUFFER_SIZE, 
             LD2450_CIRCULAR_BUFFER_SIZE * sizeof(ld2450_frame_t));
    
    return ESP_OK;
}

esp_err_t ld2450_circular_buffer_deinit(ld2450_circular_buffer_t *buffer) {
    if (buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer->consumer_mutex != NULL) {
        vSemaphoreDelete(buffer->consumer_mutex);
        buffer->consumer_mutex = NULL;
    }
    
    // Clear structure
    memset(buffer, 0, sizeof(ld2450_circular_buffer_t));
    
    return ESP_OK;
}

esp_err_t ld2450_circular_buffer_write(ld2450_circular_buffer_t *buffer, const ld2450_frame_t *frame) {
    if (buffer == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate next write index
    uint32_t next_index = buffer->write_index + 1;
    uint32_t buffer_index = get_index(next_index);
    
    // Copy frame data to circular buffer with memory barrier
    memcpy(&buffer->frames[buffer_index], frame, sizeof(ld2450_frame_t));
    
    // Memory barrier to ensure data is fully written before index is updated
    __sync_synchronize();
    
    // Update write index atomically
    buffer->write_index = next_index;
    
    // Notify all active consumers
    for (uint32_t i = 0; i < LD2450_MAX_CONSUMERS; i++) {
        if (buffer->consumers[i].active && buffer->consumers[i].task_handle != NULL) {
            // Send notification with the new frame flag
            xTaskNotify(buffer->consumers[i].task_handle, 
                       LD2450_NOTIFY_NEW_FRAME, 
                       eSetBits);
        }
    }
    
    return ESP_OK;
}

esp_err_t ld2450_circular_buffer_read_latest(ld2450_circular_buffer_t *buffer, 
                                            uint32_t consumer_id, 
                                            ld2450_frame_t *frame) {
    if (buffer == NULL || frame == NULL || consumer_id >= LD2450_MAX_CONSUMERS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!buffer->consumers[consumer_id].active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Read the current write index with memory barrier for thread safety
    uint32_t current_write_index = buffer->write_index;
    __sync_synchronize();
    
    // Check if there are any frames
    if (current_write_index == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Get the buffer index for the latest frame
    uint32_t buffer_index = get_index(current_write_index);
    
    // Copy frame data with memory barrier
    memcpy(frame, &buffer->frames[buffer_index], sizeof(ld2450_frame_t));
    
    return ESP_OK;
}

esp_err_t ld2450_circular_buffer_register_consumer(ld2450_circular_buffer_t *buffer,
                                                 TaskHandle_t task_handle,
                                                 uint32_t *consumer_id) {
    if (buffer == NULL || task_handle == NULL || consumer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_ERR_NO_MEM;
    
    if (xSemaphoreTake(buffer->consumer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take consumer mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find an available consumer slot
    for (uint32_t i = 0; i < LD2450_MAX_CONSUMERS; i++) {
        if (!buffer->consumers[i].active) {
            // Initialize consumer - only task handle and active flag
            buffer->consumers[i].task_handle = task_handle;
            buffer->consumers[i].active = true;
            
            *consumer_id = i;
            buffer->consumer_count++;
            
            ESP_LOGI(TAG, "Registered consumer %lu, total consumers: %lu", 
                     (unsigned long)i, (unsigned long)buffer->consumer_count);
            
            ret = ESP_OK;
            break;
        }
    }
    
    xSemaphoreGive(buffer->consumer_mutex);
    return ret;
}

esp_err_t ld2450_circular_buffer_unregister_consumer(ld2450_circular_buffer_t *buffer,
                                                   uint32_t consumer_id) {
    if (buffer == NULL || consumer_id >= LD2450_MAX_CONSUMERS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(buffer->consumer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take consumer mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (buffer->consumers[consumer_id].active) {
        // Clear consumer data - minimal cleanup
        buffer->consumers[consumer_id].active = false;
        buffer->consumers[consumer_id].task_handle = NULL;
        buffer->consumer_count--;
        
        ESP_LOGI(TAG, "Unregistered consumer %lu, remaining consumers: %lu", 
                 (unsigned long)consumer_id, (unsigned long)buffer->consumer_count);
    }
    
    xSemaphoreGive(buffer->consumer_mutex);
    return ESP_OK;
}
