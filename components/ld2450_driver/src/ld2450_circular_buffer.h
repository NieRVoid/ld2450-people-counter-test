/**
 * @file ld2450_circular_buffer.h
 * @brief Circular buffer implementation for LD2450 radar frames
 * 
 * Implements a lock-free circular buffer for LD2450 radar frames
 * optimized for single-producer, multi-consumer scenarios.
 * 
 * @author NieRVoid
 * @date 2025-03-15
 * @license MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "ld2450.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Buffer size must be power of 2 for optimized index calculations
 */
#define LD2450_CIRCULAR_BUFFER_SIZE 64  // 64 frames = 1024 bytes with 16-byte frames

/**
 * @brief Simplified consumer structure - producer only needs to know task handle
 */
typedef struct {
    TaskHandle_t task_handle;         /*!< Consumer task handle for notifications */
    bool active;                      /*!< Whether this consumer is active */
} ld2450_consumer_t;

/**
 * @brief Maximum number of supported consumers
 */
#define LD2450_MAX_CONSUMERS 10

/**
 * @brief Notification values for task notifications
 */
#define LD2450_NOTIFY_NEW_FRAME 0x01  /*!< New frame available notification */

/**
 * @brief Circular buffer structure
 */
typedef struct {
    ld2450_frame_t frames[LD2450_CIRCULAR_BUFFER_SIZE]; /*!< Frame storage */
    volatile uint32_t write_index;                     /*!< Current write index */
    ld2450_consumer_t consumers[LD2450_MAX_CONSUMERS]; /*!< Registered consumers */
    uint32_t consumer_count;                           /*!< Number of active consumers */
    SemaphoreHandle_t consumer_mutex;                  /*!< Mutex for consumer registration */
} ld2450_circular_buffer_t;

/**
 * @brief Initialize circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t ld2450_circular_buffer_init(ld2450_circular_buffer_t *buffer);

/**
 * @brief Deinitialize circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t ld2450_circular_buffer_deinit(ld2450_circular_buffer_t *buffer);

/**
 * @brief Write a frame to the circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @param frame Pointer to frame to write
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t ld2450_circular_buffer_write(ld2450_circular_buffer_t *buffer, const ld2450_frame_t *frame);

/**
 * @brief Read the latest frame from the circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @param consumer_id Consumer ID
 * @param frame Pointer to store the frame
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if no new frame
 */
esp_err_t ld2450_circular_buffer_read_latest(ld2450_circular_buffer_t *buffer, 
                                            uint32_t consumer_id, 
                                            ld2450_frame_t *frame);

/**
 * @brief Register a consumer with the circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @param task_handle Task handle for notifications
 * @param consumer_id Pointer to store the assigned consumer ID
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t ld2450_circular_buffer_register_consumer(ld2450_circular_buffer_t *buffer,
                                                 TaskHandle_t task_handle,
                                                 uint32_t *consumer_id);

/**
 * @brief Unregister a consumer from the circular buffer
 * 
 * @param buffer Pointer to circular buffer structure
 * @param consumer_id Consumer ID to unregister
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t ld2450_circular_buffer_unregister_consumer(ld2450_circular_buffer_t *buffer,
                                                   uint32_t consumer_id);

#ifdef __cplusplus
}
#endif
