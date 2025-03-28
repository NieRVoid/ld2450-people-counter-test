/**
 * @file people_counter_example.c
 * @brief Example application using the people counter component
 *
 * This example demonstrates:
 * - Displaying radar data every 5 seconds
 * - Showing people counter statistics every 10 seconds
 * - Receiving immediate notifications on count changes
 * - Clean exit on boot button press
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "people_counter.h"
#include "ld2450.h"

static const char *TAG = "PC_EXAMPLE";

// Button configuration
#define BOOT_BUTTON_GPIO          0
#define BUTTON_DEBOUNCE_TIME_MS   50

// Display intervals
#define RADAR_DISPLAY_INTERVAL_US     5000000  // 5 seconds in microseconds
#define COUNTER_DISPLAY_INTERVAL_US  10000000  // 10 seconds in microseconds

// Global variables for timing and program control
static volatile bool g_exit_app = false;
static int64_t g_last_button_press = 0;
static int64_t g_last_radar_display_time = 0;
static int64_t g_last_counter_display_time = 0;
static ld2450_consumer_handle_t g_radar_consumer = 0;

// GPIO interrupt handler
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t current_time = esp_timer_get_time();
    int64_t diff = current_time - g_last_button_press;
    
    // Simple debounce check
    if (diff < BUTTON_DEBOUNCE_TIME_MS * 1000) {
        return;
    }
    
    // Set exit flag on button press
    g_exit_app = true;
    g_last_button_press = current_time;
}

// Print radar target data
void print_radar_data(void) {
    ld2450_frame_t frame;
    
    // Try to get the latest frame without waiting
    esp_err_t ret = ld2450_get_latest_frame(g_radar_consumer, &frame);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get radar frame: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "---------- Radar Data Frame ----------");
    ESP_LOGI(TAG, "Number of valid targets: %d", __builtin_popcount(frame.valid_mask));
    
    for (int i = 0; i < 3; i++) {
        if (frame.valid_mask & (1 << i)) {
            const ld2450_target_t *target = &frame.targets[i];
            ESP_LOGI(TAG, "Target #%d:", i + 1);
            ESP_LOGI(TAG, "  Position: (%d, %d) mm", target->x, target->y);
            ESP_LOGI(TAG, "  Speed: %d cm/s", target->speed);
        }
    }
    ESP_LOGI(TAG, "--------------------------------------");
}

// Print people counter statistics
void print_counter_stats(void) {
    int count = 0, entries = 0, exits = 0;
    
    esp_err_t ret = people_counter_get_details(&count, &entries, &exits);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "People Count: %d (entries: %d, exits: %d)", 
                 count, entries, exits);
    } else {
        ESP_LOGW(TAG, "Failed to get people counter details: %s", esp_err_to_name(ret));
    }
}

// Callback for people counter events
static void count_changed_cb(int count, int entries, int exits, void *ctx) {
    ESP_LOGI(TAG, "PEOPLE COUNTER EVENT: %d people in room (entries: %d, exits: %d)", 
             count, entries, exits);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting People Counter Example");

    // Configure GPIO for boot button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    
    // Initialize LD2450 with default configuration
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    
    ESP_LOGI(TAG, "Initializing LD2450 with:");
    ESP_LOGI(TAG, "  UART Port: %d", radar_config.uart_port);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d", radar_config.uart_rx_pin);
    ESP_LOGI(TAG, "  TX Pin: GPIO%d", radar_config.uart_tx_pin);
    ESP_LOGI(TAG, "  Baud Rate: %" PRIu32, radar_config.uart_baud_rate);
    
    esp_err_t ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LD2450 driver initialized successfully");
    
    // Register as radar consumer to display radar data
    // Note: people_counter will register its own separate consumer
    ret = ld2450_register_consumer(&g_radar_consumer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register as radar consumer: %s", esp_err_to_name(ret));
        ld2450_deinit();
        return;
    }
    
    // Initialize the people counter module
    people_counter_config_t pc_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    pc_config.vector_threshold = 1000;         // 1m movement to count as entry/exit
    pc_config.empty_target_threshold = 5;      // 5 empty frames to consider target gone
    pc_config.detection_min_x = -2000;         // Internal detection area: 4m wide, 2m deep
    pc_config.detection_max_x = 2000;          // (does not configure radar hardware)
    pc_config.detection_min_y = 0;
    pc_config.detection_max_y = 2000;
    pc_config.count_changed_cb = count_changed_cb;  // Set callback for immediate notifications
    
    ret = people_counter_init(&pc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize people counter: %s", esp_err_to_name(ret));
        ld2450_unregister_consumer(g_radar_consumer);
        ld2450_deinit();
        return;
    }
    
    // Initialize timestamps
    int64_t current_time = esp_timer_get_time();
    g_last_radar_display_time = current_time;
    g_last_counter_display_time = current_time;
    
    ESP_LOGI(TAG, "Monitoring started. Press boot button to exit.");
    
    // Main loop
    while (!g_exit_app) {
        // Use a shorter task delay to ensure we can feed the watchdog
        vTaskDelay(pdMS_TO_TICKS(20));
        
        current_time = esp_timer_get_time();
        
        // Check if it's time to display radar data (every 5 seconds)
        if (current_time - g_last_radar_display_time >= RADAR_DISPLAY_INTERVAL_US) {
            print_radar_data();
            g_last_radar_display_time = current_time;
        }
        
        // Check if it's time to display counter stats (every 10 seconds)
        if (current_time - g_last_counter_display_time >= COUNTER_DISPLAY_INTERVAL_US) {
            print_counter_stats();
            g_last_counter_display_time = current_time;
        }
    }
    
    // Clean up before exiting
    ESP_LOGI(TAG, "Exiting program...");
    gpio_isr_handler_remove(BOOT_BUTTON_GPIO);
    ld2450_unregister_consumer(g_radar_consumer);
    people_counter_deinit();
    ld2450_deinit();
    ESP_LOGI(TAG, "Cleanup complete, goodbye!");
}