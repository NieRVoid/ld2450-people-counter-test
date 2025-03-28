/**
 * @file ld2450.c
 * @brief Main implementation of the HLK-LD2450 driver
 * 
 * This file implements the main driver functionality for the HLK-LD2450 radar sensor,
 * including initialization, deinitialization, and core operations.
 * 
 * @author NieRVoid
 * @date 2025-03-15
 * @license MIT
 */

#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ld2450.h"
#include "ld2450_private.h"
#include "ld2450_circular_buffer.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = LD2450_LOG_TAG;

// Global driver instance (singleton)
static ld2450_state_t s_ld2450_state = {0};

/**
 * @brief Get driver instance
 * 
 * @return Pointer to driver state structure
 */
ld2450_state_t *ld2450_get_instance(void)
{
    return &s_ld2450_state;
}

/**
 * @brief Initialize the LD2450 radar driver
 */
esp_err_t ld2450_init(const ld2450_config_t *config)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret = ESP_OK;
    
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (instance->initialized) {
        ESP_LOGW(TAG, "LD2450 driver already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Zero out the state structure
    memset(instance, 0, sizeof(ld2450_state_t));
    
    // Store configuration
    instance->uart_port = config->uart_port;
    instance->rx_pin = config->uart_rx_pin;
    instance->tx_pin = config->uart_tx_pin;
    instance->baud_rate = config->uart_baud_rate;
    instance->auto_processing = config->auto_processing;
    
    // Initialize circular buffer
    ret = ld2450_circular_buffer_init(&instance->circular_buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize circular buffer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create mutex for thread safety
    instance->mutex = xSemaphoreCreateMutex();
    if (!instance->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        // Clean up circular buffer that was successfully initialized
        ld2450_circular_buffer_deinit(&instance->circular_buffer);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ret = uart_driver_install(config->uart_port, LD2450_UART_RX_BUF_SIZE * 2, 
                              0, 20, &instance->uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Configure UART parameters
    ret = uart_param_config(config->uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        goto cleanup;
    }
    
    // Set UART pins
    ret = uart_set_pin(config->uart_port, config->uart_tx_pin, config->uart_rx_pin, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        goto cleanup;
    }
    
    // Configure GPIO pull-up for reliability
    gpio_set_pull_mode(config->uart_rx_pin, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(config->uart_tx_pin, GPIO_PULLUP_ONLY);
    
    // Initialize frame buffer index and sync state
    instance->frame_idx = 0;
    instance->frame_synced = false;
    instance->in_config_mode = false;
    
    // Initialize header pattern for fast detection
    static frame_header_t header_pattern;
    memcpy(header_pattern.bytes, LD2450_DATA_FRAME_HEADER, 4);
    
    // Set initialized flag
    instance->initialized = true;
    
    ESP_LOGI(TAG, "LD2450 driver initialized on UART%" PRIu32 " (RX: GPIO%" PRIu32 ", TX: GPIO%" PRIu32 ", baud: %" PRIu32 ")",  
        (uint32_t)instance->uart_port, (uint32_t)instance->rx_pin, (uint32_t)instance->tx_pin, instance->baud_rate);
    
    // Start processing task if auto-processing is enabled
    if (config->auto_processing) {
        xTaskCreate(ld2450_processing_task, "ld2450_task", LD2450_TASK_STACK_SIZE,
                    NULL, config->task_priority, &instance->task_handle);
                    
        if (!instance->task_handle) {
            ESP_LOGE(TAG, "Failed to create processing task");
            ld2450_deinit();
            return ESP_ERR_NO_MEM;
        }
        
        ESP_LOGI(TAG, "Auto-processing enabled with task priority %d", config->task_priority);
    }
    
    return ESP_OK;

cleanup:
    // Clean up resources in reverse order of allocation
    if (instance->mutex) {
        vSemaphoreDelete(instance->mutex);
        instance->mutex = NULL;
    }
    ld2450_circular_buffer_deinit(&instance->circular_buffer);
    return ret;
}

/**
 * @brief Deinitialize the LD2450 radar driver and release resources
 */
esp_err_t ld2450_deinit(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Make sure we exit configuration mode if active
    if (instance->in_config_mode) {
        ld2450_exit_config_mode();
    }
    
    // Delete task if it was created
    if (instance->task_handle) {
        instance->initialized = false;  // Signal task to exit
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
        
        // If task still running, delete it
        if (eTaskGetState(instance->task_handle) != eDeleted) {
            vTaskDelete(instance->task_handle);
        }
        
        instance->task_handle = NULL;
    }
    
    // Deinitialize circular buffer
    ld2450_circular_buffer_deinit(&instance->circular_buffer);
    
    // Delete UART driver
    uart_driver_delete(instance->uart_port);
    
    // Delete mutex
    if (instance->mutex) {
        vSemaphoreDelete(instance->mutex);
        instance->mutex = NULL;
    }
    
    // Reset state
    memset(instance, 0, sizeof(ld2450_state_t));
    
    ESP_LOGI(TAG, "LD2450 driver deinitialized");
    
    return ESP_OK;
}

/**
 * @brief Register as a consumer of radar data
 */
esp_err_t ld2450_register_consumer(ld2450_consumer_handle_t *handle)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized || !handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Use current task as consumer
    TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
    if (task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Register with circular buffer
    uint32_t consumer_id;
    esp_err_t ret = ld2450_circular_buffer_register_consumer(
        &instance->circular_buffer,
        task_handle,
        &consumer_id
    );
    
    if (ret == ESP_OK) {
        *handle = consumer_id;
    }
    
    return ret;
}

/**
 * @brief Unregister as a consumer of radar data
 */
esp_err_t ld2450_unregister_consumer(ld2450_consumer_handle_t handle)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ld2450_circular_buffer_unregister_consumer(
        &instance->circular_buffer,
        handle
    );
}

/**
 * @brief Wait for a new radar frame with timeout
 */
esp_err_t ld2450_wait_for_frame(ld2450_consumer_handle_t handle, 
                               ld2450_frame_t *frame, 
                               uint32_t timeout_ms)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized || !frame) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // First try to get latest frame without waiting
    esp_err_t ret = ld2450_get_latest_frame(handle, frame);
    if (ret == ESP_OK) {
        return ESP_OK;
    } else if (ret != ESP_ERR_NOT_FOUND) {
        return ret;  // Return actual error
    }
    
    // Wait for notification with timeout
    uint32_t notification;
    BaseType_t result = xTaskNotifyWait(
        0,                  // Don't clear bits on entry
        LD2450_NOTIFY_NEW_FRAME,  // Clear the notification bit on exit
        &notification,      // Store notification value
        pdMS_TO_TICKS(timeout_ms)
    );
    
    if (result == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // If we got notified, read the latest frame
    return ld2450_get_latest_frame(handle, frame);
}

/**
 * @brief Get the latest radar frame without waiting
 */
esp_err_t ld2450_get_latest_frame(ld2450_consumer_handle_t handle, 
                                 ld2450_frame_t *frame)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized || !frame) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ld2450_circular_buffer_read_latest(
        &instance->circular_buffer,
        handle,
        frame
    );
}

/**
 * @brief Process a radar data frame manually
 */
esp_err_t ld2450_process_frame(const uint8_t *data, size_t length, ld2450_frame_t *frame)
{
    if (!data || !frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (length != LD2450_DATA_FRAME_SIZE) {
        ESP_LOGW(TAG, "Invalid frame size: %zu bytes (expected %d)", length, LD2450_DATA_FRAME_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ld2450_parse_frame(data, length, frame);
}

/**
 * @brief Processing task for radar data
 */
void ld2450_processing_task(void *arg)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 processing task started");
    
    // Initialize for adaptive delay
    instance->idle_count = 0;
    static uint8_t data_buffer[LD2450_UART_RX_BUF_SIZE];
    
    while (instance->initialized) {
        bool processed_data = false;
        
        // Skip processing if in configuration mode
        if (instance->in_config_mode) {
            // Give longer delay while in config mode to not interfere with config commands
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Process UART events
        uart_event_t event;
        if (xQueueReceive(instance->uart_queue, &event, 0)) {
            switch (event.type) {
                case UART_DATA:
                {
                    // Read data from UART
                    int len = uart_read_bytes(instance->uart_port, data_buffer, 
                                             MIN(event.size, LD2450_UART_RX_BUF_SIZE),
                                             pdMS_TO_TICKS(10));
                    
                    if (len > 0) {
                        // Process the received data using optimized handler
                        ld2450_uart_event_handler(data_buffer, len);
                        processed_data = true;
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow detected");
                    uart_flush_input(instance->uart_port);
                    xQueueReset(instance->uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(instance->uart_port);
                    xQueueReset(instance->uart_queue);
                    break;
                case UART_BREAK:
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_DATA_BREAK:
                case UART_PATTERN_DET:
                    // Log and ignore these events
                    ESP_LOGD(TAG, "UART event: %d", event.type);
                    break;
                default:
                    ESP_LOGD(TAG, "Unhandled UART event: %d", event.type);
                    break;
            }
        }
        
        // Adaptive delay based on activity
        if (processed_data) {
            instance->idle_count = 0;
            // Yield immediately to process more data
            taskYIELD();
        } else {
            instance->idle_count++;
            // Adaptive delay based on activity (max 50ms)
            uint32_t delay_ms = MIN(instance->idle_count, 10) * 5;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    ESP_LOGI(TAG, "LD2450 processing task stopped");
    vTaskDelete(NULL);
}

// Configuration functions remain in ld2450_config.c