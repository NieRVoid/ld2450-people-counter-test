/**
 * @file people_counter_example.c
 * @brief Basic example of using people counter with external radar initialization
 *
 * This example shows the recommended way to use the people counter module
 * with external radar initialization.
 *
 * @author NieRVoid
 * @date 2025-03-14
 * @license MIT
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "people_counter.h"
#include "ld2450.h"

static const char *TAG = "PC_EXAMPLE";

// Callback function for count changes
static void count_changed_cb(int count, int entries, int exits, void *ctx)
{
    ESP_LOGI(TAG, "People count changed: %d in room (entries: %d, exits: %d)", 
             count, entries, exits);
}

void app_main(void)
{
    ESP_LOGI(TAG, "People Counter Example Starting");

    // 1. First initialize the LD2450 radar driver
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    radar_config.uart_rx_pin = 16;
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    
    esp_err_t ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize radar: %s", esp_err_to_name(ret));
        return;
    }
    
    // 2. Then initialize the people counter module
    people_counter_config_t pc_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    pc_config.vector_threshold = 1000;          // 100cm movement to count as entry/exit
    pc_config.empty_target_threshold = 5;      // 5 empty frames to consider target gone
    pc_config.detection_min_x = -2000;         // Detection area: 4m wide, 2m deep
    pc_config.detection_max_x = 2000;
    pc_config.detection_min_y = 0;
    pc_config.detection_max_y = 2000;
    pc_config.count_changed_cb = count_changed_cb;
    
    ret = people_counter_init(&pc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize people counter: %s", esp_err_to_name(ret));
        ld2450_deinit(); // Clean up radar if people counter initialization fails
        return;
    }

    // Main application loop
    while (1) {
        int count, entries, exits;
        if (people_counter_get_details(&count, &entries, &exits) == ESP_OK) {
            ESP_LOGI(TAG, "Current status: %d people in room (entries: %d, exits: %d)", 
                     count, entries, exits);
        }
        
        // Sleep for 2 seconds before next status report
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // These would typically be in a cleanup function
    people_counter_deinit();
    ld2450_deinit();
}