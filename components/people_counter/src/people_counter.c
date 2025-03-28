/**
 * @file people_counter.c
 * @brief Implementation of people counting module using LD2450 radar sensor
 *
 * This module implements a people counting algorithm that monitors x-axis
 * movements to determine entries and exits through a doorway.
 *
 * @author NieRVoid
 * @date 2025-03-14
 * @license MIT
 */

#include <string.h>
#include "people_counter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

static const char *TAG = "PEOPLE_COUNTER";

// Task configuration
#define PC_TASK_STACK_SIZE    4096
#define PC_TASK_PRIORITY      5
#define PC_FRAME_WAIT_TIMEOUT pdMS_TO_TICKS(500) // Maximum wait time for new frames

/**
 * @brief Target tracking structure
 */
typedef struct {
    int16_t initial_x;          // Initial X position when target first detected
    int16_t current_x;          // Current X position
    int16_t vector_length;      // Current vector length (current_x - initial_x)
    bool counted;               // Flag indicating if this trajectory has been counted
    bool active;                // Flag indicating if target is currently active
    uint8_t empty_count;        // Counter for consecutive empty reports
    int64_t first_seen_time;    // Timestamp when target first detected (µs)
    int64_t last_update_time;   // Last update timestamp (µs)
} pc_target_t;

/**
 * @brief Copy of target data for processing without mutex
 */
typedef struct {
    int16_t x;                  // Target's X position
    int16_t y;                  // Target's Y position
    bool in_area;               // Whether target is in detection area
} pc_target_data_t;

/**
 * @brief Module context structure
 */
typedef struct {
    int count;                  // Current people count (entries - exits)
    int total_entries;          // Total entries counted
    int total_exits;            // Total exits counted
    pc_target_t targets[3];     // Array of tracked targets (max 3 from LD2450)
    people_counter_config_t config;
    bool initialized;
    
    // Use atomic operations instead of mutex for simple counters
    // Keep mutex only for more complex operations
    SemaphoreHandle_t mutex;    // For thread safety on complex operations
    
    ld2450_consumer_handle_t consumer_handle; // Handle for radar consumer
    TaskHandle_t task_handle;   // Handle for the data collection task
    bool running;               // Flag to control task execution
} people_counter_context_t;

// Global instance of the counter context
static people_counter_context_t s_pc_context = {0};

// Forward declarations of static functions
static bool is_in_detection_area(int16_t x, int16_t y, const people_counter_config_t *config);
static void process_radar_frame(const ld2450_frame_t *frame);
static void people_counter_task(void *pvParameters);
static void notify_count_change(int count, int entries, int exits, void (*callback)(int, int, int, void*), void* user_context);

/**
 * @brief Initialize people counter with configuration
 */
esp_err_t people_counter_init(const people_counter_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_pc_context.initialized) {
        ESP_LOGW(TAG, "People counter already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize context with defaults
    memset(&s_pc_context, 0, sizeof(s_pc_context));
    memcpy(&s_pc_context.config, config, sizeof(people_counter_config_t));

    // Create mutex for thread safety
    s_pc_context.mutex = xSemaphoreCreateMutex();
    if (s_pc_context.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Register as a consumer with the LD2450 driver
    esp_err_t ret = ld2450_register_consumer(&s_pc_context.consumer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register as radar consumer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_pc_context.mutex);
        s_pc_context.mutex = NULL;
        return ret;
    }

    // Mark as running before starting the task
    s_pc_context.running = true;
    s_pc_context.initialized = true;

    // Create task to wait for and process radar data
    BaseType_t task_created = xTaskCreate(
        people_counter_task,
        "people_counter",
        PC_TASK_STACK_SIZE,
        NULL,
        PC_TASK_PRIORITY,
        &s_pc_context.task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create people counter task");
        ld2450_unregister_consumer(s_pc_context.consumer_handle);
        vSemaphoreDelete(s_pc_context.mutex);
        s_pc_context.mutex = NULL;
        s_pc_context.initialized = false;
        s_pc_context.running = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "People counter initialized successfully");
    ESP_LOGI(TAG, "Detection area configured: X(%d,%d) Y(%d,%d) mm",
             s_pc_context.config.detection_min_x, s_pc_context.config.detection_max_x,
             s_pc_context.config.detection_min_y, s_pc_context.config.detection_max_y);

    return ESP_OK;
}

/**
 * @brief Deinitialize and release resources
 */
esp_err_t people_counter_deinit(void)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop the task
    s_pc_context.running = false;
    
    // Give task time to exit cleanly
    if (s_pc_context.task_handle != NULL) {
        // Unblock any waiting operations
        xTaskNotify(s_pc_context.task_handle, 0, eNoAction);
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(s_pc_context.task_handle);
        s_pc_context.task_handle = NULL;
    }

    // Unregister consumer
    if (s_pc_context.consumer_handle != 0) {
        ld2450_unregister_consumer(s_pc_context.consumer_handle);
        s_pc_context.consumer_handle = 0;
    }

    // Delete mutex
    if (s_pc_context.mutex != NULL) {
        vSemaphoreDelete(s_pc_context.mutex);
        s_pc_context.mutex = NULL;
    }

    s_pc_context.initialized = false;
    ESP_LOGI(TAG, "People counter deinitialized");

    return ESP_OK;
}

/**
 * @brief Update configuration parameters
 */
esp_err_t people_counter_update_config(const people_counter_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create a local copy of the configuration first
    people_counter_config_t new_config;
    memcpy(&new_config, config, sizeof(people_counter_config_t));

    // Use a limited timeout for mutex operations
    if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex when updating config");
        return ESP_ERR_TIMEOUT;
    }

    // Update configuration parameters
    memcpy(&s_pc_context.config, &new_config, sizeof(people_counter_config_t));
    
    xSemaphoreGive(s_pc_context.mutex);

    ESP_LOGI(TAG, "Configuration updated with vector threshold: %d mm", config->vector_threshold);
    ESP_LOGI(TAG, "Updated detection area: X(%d,%d) Y(%d,%d) mm",
             config->detection_min_x, config->detection_max_x,
             config->detection_min_y, config->detection_max_y);

    return ESP_OK;
}

/**
 * @brief Get current people count
 */
int people_counter_get_count(void)
{
    if (!s_pc_context.initialized) {
        ESP_LOGW(TAG, "People counter not initialized");
        return 0;
    }

    // Read the current count value atomically
    return s_pc_context.count;
}

/**
 * @brief Get detailed counts
 */
esp_err_t people_counter_get_details(int *count, int *entries, int *exits)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (count == NULL && entries == NULL && exits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy values atomically without requiring a mutex
    if (count != NULL) {
        *count = s_pc_context.count;
    }
    if (entries != NULL) {
        *entries = s_pc_context.total_entries;
    }
    if (exits != NULL) {
        *exits = s_pc_context.total_exits;
    }
    
    return ESP_OK;
}

/**
 * @brief Reset counter to zero
 */
esp_err_t people_counter_reset(void)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex with timeout
    if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex when resetting counter");
        return ESP_ERR_TIMEOUT;
    }

    // Reset counters
    int old_count = s_pc_context.count;
    s_pc_context.count = 0;
    s_pc_context.total_entries = 0;
    s_pc_context.total_exits = 0;
    
    // Reset all target tracking
    for (int i = 0; i < 3; i++) {
        s_pc_context.targets[i].active = false;
        s_pc_context.targets[i].counted = false;
        s_pc_context.targets[i].empty_count = 0;
    }
    
    // Get callback outside mutex to avoid issues
    void (*callback)(int, int, int, void*) = s_pc_context.config.count_changed_cb;
    void *user_context = s_pc_context.config.user_context;
    
    xSemaphoreGive(s_pc_context.mutex);
    
    ESP_LOGI(TAG, "People counter reset to zero (was: %d)", old_count);
    
    // Notify about change if needed
    if (old_count != 0 && callback != NULL) {
        // Call callback safely outside the mutex
        notify_count_change(0, 0, 0, callback, user_context);
    }
    
    return ESP_OK;
}

/**
 * @brief Register callback for count changes
 */
esp_err_t people_counter_register_callback(
    void (*callback)(int count, int entries, int exits, void *context), 
    void *user_context)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex with timeout
    if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex when registering callback");
        return ESP_ERR_TIMEOUT;
    }

    s_pc_context.config.count_changed_cb = callback;
    s_pc_context.config.user_context = user_context;
    
    xSemaphoreGive(s_pc_context.mutex);
    
    ESP_LOGI(TAG, "Callback %sregistered", callback ? "" : "un");
    return ESP_OK;
}

/**
 * @brief Check if target is in detection area
 */
static bool is_in_detection_area(int16_t x, int16_t y, const people_counter_config_t *config)
{
    return (x >= config->detection_min_x && 
            x <= config->detection_max_x &&
            y >= config->detection_min_y && 
            y <= config->detection_max_y);
}

/**
 * @brief Safely notify about count changes
 */
static void notify_count_change(int count, int entries, int exits, 
                               void (*callback)(int, int, int, void*), 
                               void* user_context)
{
    if (callback) {
        ESP_LOGD(TAG, "Notifying count change: %d (%d/%d)", count, entries, exits);
        callback(count, entries, exits, user_context);
    }
}

/**
 * @brief Process radar frame data and update tracking
 *
 * This completely redesigned function avoids mutex contention by:
 * 1. Getting a quick snapshot of state with minimal mutex holding
 * 2. Processing the data without holding the mutex
 * 3. Only updating state under mutex when changes are needed
 * 4. Never calling user callbacks while holding the mutex
 */
static void process_radar_frame(const ld2450_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }
    
    int64_t current_timestamp = esp_timer_get_time();
    people_counter_config_t config_copy;
    pc_target_data_t target_data[3] = {0};
    bool targets_active[3] = {0};
    
    // STEP 1: Take a minimal snapshot of current state
    if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Couldn't get mutex for frame processing - skipping frame");
        return;
    }

    // Make a local copy of the configuration
    memcpy(&config_copy, &s_pc_context.config, sizeof(people_counter_config_t));
    
    // Copy target active states and get target data
    for (int i = 0; i < 3; i++) {
        targets_active[i] = s_pc_context.targets[i].active;
        
        if (frame->valid_mask & (1 << i)) {
            target_data[i].x = frame->targets[i].x;
            target_data[i].y = frame->targets[i].y;
            target_data[i].in_area = is_in_detection_area(
                frame->targets[i].x, frame->targets[i].y, &config_copy);
        }
    }
    
    xSemaphoreGive(s_pc_context.mutex);

    // STEP 2: Process each target without holding mutex
    bool changes_needed = false;
    bool count_changed = false;
    int new_entries = 0, new_exits = 0;
    
    for (int i = 0; i < 3; i++) {
        // STEP 3: Only re-acquire mutex when changes are needed
        if ((frame->valid_mask & (1 << i)) && target_data[i].in_area) {
            // Target is valid in this frame and in our area
            bool is_new_target = !targets_active[i];
            
            if (is_new_target) {
                // Need to properly initialize the target
                changes_needed = true;
            } else {
                // We need to check for potential entry/exit
                if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
                    continue; // Couldn't get mutex, skip this target for now
                }
                
                pc_target_t *target = &s_pc_context.targets[i];
                
                // Check if this target has already been counted
                if (!target->counted) {
                    // Update position
                    target->current_x = target_data[i].x;
                    target->last_update_time = current_timestamp;
                    target->empty_count = 0;
                    
                    // Calculate vector and check if threshold exceeded
                    int16_t vector = target_data[i].x - target->initial_x;
                    target->vector_length = vector;
                    
                    if (abs(vector) >= config_copy.vector_threshold) {
                        // Mark as counted
                        target->counted = true;
                        
                        if (vector > 0) {
                            // Entry
                            s_pc_context.count++;
                            s_pc_context.total_entries++;
                            new_entries++;
                            count_changed = true;
                            ESP_LOGI(TAG, "Entry detected. Current count: %d", s_pc_context.count);
                        } else {
                            // Exit
                            s_pc_context.count--;
                            s_pc_context.total_exits++;
                            new_exits++;
                            count_changed = true;
                            ESP_LOGI(TAG, "Exit detected. Current count: %d", s_pc_context.count);
                        }
                    }
                }
                
                xSemaphoreGive(s_pc_context.mutex);
            }
        } else if (targets_active[i]) {
            // Target was active but now missing - increment empty count
            changes_needed = true;
        }
    }
    
    // STEP 4: Make necessary updates under mutex but keep it short
    if (changes_needed) {
        if (xSemaphoreTake(s_pc_context.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                if ((frame->valid_mask & (1 << i)) && target_data[i].in_area) {
                    if (!s_pc_context.targets[i].active) {
                        // Initialize new target
                        s_pc_context.targets[i].active = true;
                        s_pc_context.targets[i].initial_x = target_data[i].x;
                        s_pc_context.targets[i].current_x = target_data[i].x;
                        s_pc_context.targets[i].first_seen_time = current_timestamp;
                        s_pc_context.targets[i].last_update_time = current_timestamp;
                        s_pc_context.targets[i].counted = false;
                        s_pc_context.targets[i].empty_count = 0;
                        s_pc_context.targets[i].vector_length = 0;
                        
                        ESP_LOGD(TAG, "New target %d detected at position (%d, %d)", 
                                i, target_data[i].x, target_data[i].y);
                    }
                } else if (s_pc_context.targets[i].active) {
                    // Target was active but now missing or outside area
                    s_pc_context.targets[i].empty_count++;
                    
                    if (s_pc_context.targets[i].empty_count >= config_copy.empty_target_threshold) {
                        // Reset target tracking
                        ESP_LOGD(TAG, "Target %d lost after %d empty frames", 
                                 i, s_pc_context.targets[i].empty_count);
                        s_pc_context.targets[i].active = false;
                    }
                }
            }
            xSemaphoreGive(s_pc_context.mutex);
        }
    }
    
    // STEP 5: Notify about count changes outside any mutex
    if (count_changed && config_copy.count_changed_cb != NULL) {
        notify_count_change(s_pc_context.count, 
                          s_pc_context.total_entries, 
                          s_pc_context.total_exits,
                          config_copy.count_changed_cb,
                          config_copy.user_context);
    }
}

/**
 * @brief Task function to efficiently wait for and process radar data
 */
static void people_counter_task(void *pvParameters)
{
    ESP_LOGI(TAG, "People counter task started");
    
    ld2450_frame_t frame;
    esp_err_t ret;
    
    // Main processing loop
    while (s_pc_context.running) {
        // Wait for new frame with timeout
        ret = ld2450_wait_for_frame(s_pc_context.consumer_handle, &frame, PC_FRAME_WAIT_TIMEOUT);
        
        if (ret == ESP_OK) {
            // Process the frame data
            process_radar_frame(&frame);
            
            // Yield to other tasks after processing
            vTaskDelay(1);
        } else if (ret != ESP_ERR_TIMEOUT) {
            // Only log errors that aren't timeouts
            ESP_LOGW(TAG, "Error waiting for radar frame: %s", esp_err_to_name(ret));
            // Add a short delay before retrying to avoid spamming logs
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            // On timeout, briefly yield to other tasks
            vTaskDelay(1);
        }
    }
    
    ESP_LOGI(TAG, "People counter task stopped");
    vTaskDelete(NULL);
}