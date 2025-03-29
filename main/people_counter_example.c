/**
 * @file people_counter_example.c
 * @brief Example application using the people counter component
 *
 * This example demonstrates:
 * - Displaying radar data every 5 seconds
 * - Showing people counter statistics every 10 seconds
 * - Receiving immediate notifications on count changes
 * - Clean exit on boot button press
 * - Resource monitoring every 30 seconds
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "protocol_examples_common.h"

#include "ld2450.h"
#include "people_counter.h"
// #include "occupancy_manager.h"

#include "resource_monitor.h"

static const char *TAG = "PC_EXAMPLE";

// Button configuration
#define BOOT_BUTTON_GPIO          0
#define BUTTON_DEBOUNCE_TIME_MS   50

// Display intervals in milliseconds
#define RADAR_DISPLAY_INTERVAL_MS    5000  // 5 seconds
#define COUNTER_DISPLAY_INTERVAL_MS  10000  // 10 seconds
#define RESOURCE_MONITOR_INTERVAL_MS 30000  // 30 seconds

// Event bits
#define EVENT_EXIT_APP            (1 << 0)
#define EVENT_DISPLAY_RADAR       (1 << 1)
#define EVENT_DISPLAY_COUNTER     (1 << 2)

// Global handles
static EventGroupHandle_t g_event_group = NULL;
static TimerHandle_t g_radar_timer = NULL;
static TimerHandle_t g_counter_timer = NULL;
static ld2450_consumer_handle_t g_radar_consumer = 0;
static int64_t g_last_button_press = 0;

// Function prototypes
static void print_radar_data(void);
static void print_counter_stats(void);
static void count_changed_cb(int count, int entries, int exits, void *ctx);
static esp_err_t init_networking(void);
static void cleanup_resources(void);
static esp_err_t init_hardware(void);

// ISR handler for button press
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t current_time = esp_timer_get_time();
    int64_t diff = current_time - g_last_button_press;
    
    // Simple debounce check
    if (diff < BUTTON_DEBOUNCE_TIME_MS * 1000) {
        return;
    }
    
    g_last_button_press = current_time;
    
    // Set event bit from ISR context
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_event_group != NULL) {
        xEventGroupSetBitsFromISR(g_event_group, EVENT_EXIT_APP, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// Timer callback for radar display
static void radar_timer_callback(TimerHandle_t xTimer) {
    if (g_event_group != NULL) {
        xEventGroupSetBits(g_event_group, EVENT_DISPLAY_RADAR);
    }
}

// Timer callback for counter display
static void counter_timer_callback(TimerHandle_t xTimer) {
    if (g_event_group != NULL) {
        xEventGroupSetBits(g_event_group, EVENT_DISPLAY_COUNTER);
    }
}

// Print radar target data
static void print_radar_data(void) {
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
static void print_counter_stats(void) {
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

/**
 * @brief Initialize networking stack and connect to WiFi
 */
static esp_err_t init_networking(void) {
    ESP_LOGI(TAG, "Initializing networking stack");
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = example_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Connected to WiFi network");
    return ESP_OK;
}

/**
 * @brief Initialize hardware and required components
 */
static esp_err_t init_hardware(void) {
    esp_err_t ret;
    
    // Configure GPIO for boot button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install GPIO ISR handler
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize LD2450 with default configuration
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    
    ESP_LOGI(TAG, "Initializing LD2450 with:");
    ESP_LOGI(TAG, "  UART Port: %d", radar_config.uart_port);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d", radar_config.uart_rx_pin);
    ESP_LOGI(TAG, "  TX Pin: GPIO%d", radar_config.uart_tx_pin);
    ESP_LOGI(TAG, "  Baud Rate: %" PRIu32, radar_config.uart_baud_rate);
    
    ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LD2450 driver initialized successfully");
    
    // Register as radar consumer to display radar data
    ret = ld2450_register_consumer(&g_radar_consumer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register as radar consumer: %s", esp_err_to_name(ret));
        ld2450_deinit();
        return ret;
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
        return ret;
    }
    
    // Initialize resource monitor
    ESP_LOGI(TAG, "Initializing resource monitor...");
    ret = resource_monitor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize resource monitor: %s", esp_err_to_name(ret));
        // Non-critical, continue anyway
    } else {
        // Start periodic resource monitoring
        ret = resource_monitor_start_periodic(RESOURCE_MONITOR_INTERVAL_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start periodic resource monitoring: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Resource monitoring started with %dms interval", 
                     RESOURCE_MONITOR_INTERVAL_MS);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Cleanup resources before exiting
 */
static void cleanup_resources(void) {
    ESP_LOGI(TAG, "Cleaning up resources...");
    
    // Stop timers first
    if (g_radar_timer != NULL) {
        xTimerStop(g_radar_timer, portMAX_DELAY);
        xTimerDelete(g_radar_timer, portMAX_DELAY);
        g_radar_timer = NULL;
    }
    
    if (g_counter_timer != NULL) {
        xTimerStop(g_counter_timer, portMAX_DELAY);
        xTimerDelete(g_counter_timer, portMAX_DELAY);
        g_counter_timer = NULL;
    }
    
    // First unregister consumers to stop data flow
    if (g_radar_consumer != 0) {
        ld2450_unregister_consumer(g_radar_consumer);
        g_radar_consumer = 0;
    }
    
    // Short delay to ensure no more data flowing through
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Deinitialize people counter
    people_counter_deinit();
    
    // Additional delay to ensure complete cleanup
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Deinitialize resource monitor
    resource_monitor_stop_periodic();
    resource_monitor_deinit();
    
    // Finally deinitialize the radar
    ld2450_deinit();
    
    // Remove GPIO ISR handler
    gpio_isr_handler_remove(BOOT_BUTTON_GPIO);
    
    // Delete event group
    if (g_event_group != NULL) {
        vEventGroupDelete(g_event_group);
        g_event_group = NULL;
    }
    
    ESP_LOGI(TAG, "Cleanup complete, exiting application, goodbye!");
}

/**
 * @brief Main event handling task
 */
static void event_handling_task(void *pvParameters) {
    ESP_LOGI(TAG, "Event handling task started");
    
    while (1) {
        // Wait for any events with a timeout to ensure watchdog is fed
        EventBits_t bits = xEventGroupWaitBits(
            g_event_group,
            EVENT_EXIT_APP | EVENT_DISPLAY_RADAR | EVENT_DISPLAY_COUNTER,
            pdTRUE,  // Clear bits before returning
            pdFALSE, // Don't wait for all bits
            pdMS_TO_TICKS(500)
        );
        
        // Check exit condition first
        if (bits & EVENT_EXIT_APP) {
            ESP_LOGI(TAG, "Exit event received");
            break;
        }
        
        // Handle radar display event
        if (bits & EVENT_DISPLAY_RADAR) {
            print_radar_data();
        }
        
        // Handle counter display event
        if (bits & EVENT_DISPLAY_COUNTER) {
            print_counter_stats();
        }
    }
    
    // Cleanup and exit
    cleanup_resources();
    
    ESP_LOGI(TAG, "Event handling task exiting");
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting People Counter Example");

    // Create event group for synchronization
    g_event_group = xEventGroupCreate();
    if (g_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    // Initialize networking stack
    esp_err_t ret = init_networking();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize networking: %s", esp_err_to_name(ret));
        vEventGroupDelete(g_event_group);
        g_event_group = NULL;
        return;
    }
    
    // Initialize hardware components
    ret = init_hardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed: %s", esp_err_to_name(ret));
        vEventGroupDelete(g_event_group);
        g_event_group = NULL;
        return;
    }
    
    // Create periodic timers
    g_radar_timer = xTimerCreate(
        "radar_timer", 
        pdMS_TO_TICKS(RADAR_DISPLAY_INTERVAL_MS),
        pdTRUE,   // Auto reload
        NULL,     // Timer ID
        radar_timer_callback
    );
    
    g_counter_timer = xTimerCreate(
        "counter_timer", 
        pdMS_TO_TICKS(COUNTER_DISPLAY_INTERVAL_MS),
        pdTRUE,   // Auto reload
        NULL,     // Timer ID
        counter_timer_callback
    );
    
    if (g_radar_timer == NULL || g_counter_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timers");
        cleanup_resources();
        return;
    }
    
    // Start timers
    if (xTimerStart(g_radar_timer, portMAX_DELAY) != pdPASS ||
        xTimerStart(g_counter_timer, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timers");
        cleanup_resources();
        return;
    }
    
    // Create event handling task
    TaskHandle_t event_task_handle;
    BaseType_t task_created = xTaskCreate(
        event_handling_task,
        "event_handler",
        4096,
        NULL,
        5,
        &event_task_handle
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create event handling task");
        cleanup_resources();
        return;
    }
    
    ESP_LOGI(TAG, "System initialized and running");
    ESP_LOGI(TAG, "Press the boot button to exit the application");
    
    // Print initial counter stats
    print_counter_stats();
}