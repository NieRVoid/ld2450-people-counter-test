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
#define DOUBLE_PRESS_INTERVAL_MS  500

// Global variables for button state and program control
static volatile bool g_exit_app = false;
static int64_t g_last_button_press = 0;
static int64_t g_last_frame_time = 0;

// GPIO interrupt handler
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t current_time = esp_timer_get_time();
    int64_t diff = current_time - g_last_button_press;
    
    // Debounce check
    if (diff < BUTTON_DEBOUNCE_TIME_MS * 1000) {
        return;
    }
    
    // Check for double press
    if (diff < DOUBLE_PRESS_INTERVAL_MS * 1000) {
        g_exit_app = true;
    }
    
    g_last_button_press = current_time;
}

// Utility function to print buffer as hex dump
void print_hex_dump(const char* prefix, const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        ESP_LOGI(TAG, "%s: <empty buffer>", prefix);
        return;
    }
    
    ESP_LOGI(TAG, "%s (%u bytes):", prefix, len);
    
    char line[128];
    char *ptr = line;
    
    for (size_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            if (i > 0) {
                *ptr = '\0';
                ESP_LOGI(TAG, "%s", line);
            }
            ptr = line;
            ptr += sprintf(ptr, "  %04x: ", (unsigned int)i);
        }
        
        ptr += sprintf(ptr, "%02x ", data[i]);
        
        // Add extra space after 8 bytes for readability
        if ((i % 16 == 7)) {
            ptr += sprintf(ptr, " ");
        }
    }
    
    // Print any remaining bytes
    if (ptr != line) {
        *ptr = '\0';
        ESP_LOGI(TAG, "%s", line);
    }
}

// Function to retrieve and print error debug info from driver
void print_error_debug_info(void) {
    uint8_t debug_buffer[256];
    size_t debug_len = 0;
    
    esp_err_t ret = ld2450_get_last_error_data(debug_buffer, sizeof(debug_buffer), &debug_len);
    if (ret == ESP_OK && debug_len > 0) {
        print_hex_dump("Last error data", debug_buffer, debug_len);
    } else {
        ESP_LOGI(TAG, "No error debug data available");
    }
}

// Simplified callback function that doesn't duplicate logging functionality
static void radar_data_callback(const ld2450_frame_t *frame, void *user_ctx) {
    if (!frame) return;
    
    // Just update the last frame time
    g_last_frame_time = frame->timestamp;
    
    // Optional: Track target count changes (as a simple example of application logic)
    static int prev_target_count = -1;
    if (frame->count != prev_target_count) {
        ESP_LOGI(TAG, "Target count changed: %d → %d", prev_target_count, frame->count);
        prev_target_count = frame->count;
    }
}

void print_region_filter(ld2450_filter_type_t type, ld2450_region_t regions[3]) {
    const char *type_str = "Unknown";
    switch (type) {
        case LD2450_FILTER_DISABLED: type_str = "Disabled"; break;
        case LD2450_FILTER_INCLUDE_ONLY: type_str = "Include Only"; break;
        case LD2450_FILTER_EXCLUDE: type_str = "Exclude"; break;
    }
    
    ESP_LOGI(TAG, "Region Filter: %s", type_str);
    if (type != LD2450_FILTER_DISABLED) {
        for (int i = 0; i < 3; i++) {
            ESP_LOGI(TAG, "  Region %d: (%d,%d) to (%d,%d) mm", 
                    i+1, regions[i].x1, regions[i].y1, regions[i].x2, regions[i].y2);
        }
    }
}

// Callback function for count changes
static void count_changed_cb(int count, int entries, int exits, void *ctx)
{
    ESP_LOGI(TAG, "People count changed: %d in room (entries: %d, exits: %d)", 
             count, entries, exits);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting LD2450 Radar Test");
    
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
    ld2450_config_t config = LD2450_DEFAULT_CONFIG();
    
    // Configure the component for verbose logging
    config.log_level = LD2450_LOG_VERBOSE;  // Enable all component logs
    config.data_log_interval_ms = 3000;     // Set to 3 seconds to match original display interval
    
    // Print configuration settings
    ESP_LOGI(TAG, "Initializing LD2450 with:");
    ESP_LOGI(TAG, "  UART Port: %d", config.uart_port);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d", config.uart_rx_pin);
    ESP_LOGI(TAG, "  TX Pin: GPIO%d", config.uart_tx_pin);
    ESP_LOGI(TAG, "  Baud Rate: %lu", config.uart_baud_rate);
    ESP_LOGI(TAG, "  Auto Processing: %s", config.auto_processing ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "  Log Level: %d (VERBOSE)", config.log_level);
    ESP_LOGI(TAG, "  Data Log Interval: %lu ms", config.data_log_interval_ms);
    
    esp_err_t ret = ld2450_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
        return;
    }
    ESP_LOGI(TAG, "LD2450 initialized successfully");
    
    // Initialize the people counter module
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
    
    // Restart radar module
    ESP_LOGI(TAG, "Restarting radar module...");
    ret = ld2450_restart_module();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart module! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    // Wait for the module to restart
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Radar module restarted");
    
    // Get configuration information
    ESP_LOGI(TAG, "\n======== RADAR CONFIGURATION ========");

    // Get region filter configuration
    ld2450_filter_type_t filter_type;
    ld2450_region_t regions[3];
    ret = ld2450_get_region_filter(&filter_type, regions);
    if (ret == ESP_OK) {
        print_region_filter(filter_type, regions);
    } else {
        ESP_LOGE(TAG, "Failed to get region filter! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    ESP_LOGI(TAG, "===================================\n");
    
    // Initialize last frame time
    g_last_frame_time = esp_timer_get_time();
    
    // Wait for and process radar data
    ESP_LOGI(TAG, "Waiting for radar detection data...");
    ESP_LOGI(TAG, "(Press boot button twice quickly to exit)");
    
    // Variables for periodic count display
    int64_t last_count_display = esp_timer_get_time();
    const int64_t count_display_interval = 5000000; // 5 seconds in microseconds
    
    // Main loop - the component will handle logging
    while (!g_exit_app) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Check exit flag every 100ms
        
        // Check if it's time to display the current count (every 5 seconds)
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_count_display >= count_display_interval) {
            // Get current counts
            int current_count, entries, exits;
            esp_err_t ret = people_counter_get_details(&current_count, &entries, &exits);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Current people count: %d (entries: %d, exits: %d)", 
                         current_count, entries, exits);
            } else {
                ESP_LOGE(TAG, "Failed to get people count: %s", esp_err_to_name(ret));
            }
            
            // Update last display time
            last_count_display = current_time;
        }
    }
    
    // Clean up before exiting
    ESP_LOGI(TAG, "Exiting program...");
    gpio_isr_handler_remove(BOOT_BUTTON_GPIO);
    people_counter_deinit();
    ld2450_deinit();
    ESP_LOGI(TAG, "Cleanup complete, goodbye!");
}