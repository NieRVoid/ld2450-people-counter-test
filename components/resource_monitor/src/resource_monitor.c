/**
 * @file resource_monitor.c
 * @brief Resource monitor implementation
 */

#include "resource_monitor.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_chip_info.h"
#include <string.h>
#include <inttypes.h>

// For flash filesystem stats
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

static const char *TAG = "resource-monitor";

// Configuration validation
#if configGENERATE_RUN_TIME_STATS == 0
#error "Resource monitor requires configGENERATE_RUN_TIME_STATS to be enabled in FreeRTOSConfig.h"
#endif

#if configUSE_TRACE_FACILITY == 0 
#error "Resource monitor requires configUSE_TRACE_FACILITY to be enabled in FreeRTOSConfig.h"
#endif

// Constants
#define RM_MIN_INTERVAL_MS     100
#define RM_TASK_INFO_BUFFER_SIZE 1024
#define RM_PRINT_BUFFER_SIZE    1024
#define RM_MAX_TASK_COUNT       32

// Internal context
typedef struct {
    bool initialized;
    bool periodic_running;
    TimerHandle_t periodic_timer;
} resource_monitor_ctx_t;

static resource_monitor_ctx_t s_rm_ctx = {
    .initialized = false,
    .periodic_running = false,
    .periodic_timer = NULL
};

// Forward declarations
static void periodic_timer_callback(TimerHandle_t timer);

esp_err_t resource_monitor_init(void)
{
    if (s_rm_ctx.initialized) {
        ESP_LOGD(TAG, "Resource monitor already initialized");
        return ESP_OK; // Already initialized
    }
    
    s_rm_ctx.initialized = true;
    ESP_LOGI(TAG, "Resource monitor initialized");
    
    return ESP_OK;
}

esp_err_t resource_monitor_deinit(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_OK; // Not initialized
    }
    
    // Stop periodic timer if running
    if (s_rm_ctx.periodic_running) {
        resource_monitor_stop_periodic();
    }
    
    // Delete timer if it exists
    if (s_rm_ctx.periodic_timer != NULL) {
        if (xTimerDelete(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGW(TAG, "Failed to delete periodic timer");
        }
        s_rm_ctx.periodic_timer = NULL;
    }
    
    s_rm_ctx.initialized = false;
    ESP_LOGI(TAG, "Resource monitor deinitialized");
    
    return ESP_OK;
}

esp_err_t resource_monitor_get_mem_stats(resource_monitor_mem_stats_t *stats)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear stats structure
    memset(stats, 0, sizeof(resource_monitor_mem_stats_t));
    
    // Get internal heap stats
    stats->free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    stats->total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    stats->min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    stats->max_alloc_heap = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
#ifdef CONFIG_SPIRAM
    // Get PSRAM stats if enabled
    stats->free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    stats->total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    stats->min_free_psram = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    stats->max_alloc_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
#endif
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_mem_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    resource_monitor_mem_stats_t stats;
    esp_err_t err = resource_monitor_get_mem_stats(&stats);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get memory stats: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "==== MEMORY STATISTICS ====");
    ESP_LOGI(TAG, "Internal Heap: Free: %" PRIu32 " bytes, Total: %" PRIu32 " bytes, Min Free: %" PRIu32 " bytes, Max Block: %" PRIu32 " bytes",
             (uint32_t)stats.free_heap, (uint32_t)stats.total_heap, (uint32_t)stats.min_free_heap, (uint32_t)stats.max_alloc_heap);
    
    if (stats.total_heap > 0) {
        ESP_LOGI(TAG, "Internal Heap Usage: %" PRIu32 "%% used, %" PRIu32 "%% free", 
                (uint32_t)((stats.total_heap - stats.free_heap) * 100 / stats.total_heap),
                (uint32_t)(stats.free_heap * 100 / stats.total_heap));
    }
             
#ifdef CONFIG_SPIRAM
    ESP_LOGI(TAG, "PSRAM: Free: %" PRIu32 " bytes, Total: %" PRIu32 " bytes, Min Free: %" PRIu32 " bytes, Max Block: %" PRIu32 " bytes",
             (uint32_t)stats.free_psram, (uint32_t)stats.total_psram, (uint32_t)stats.min_free_psram, (uint32_t)stats.max_alloc_psram);
    if (stats.total_psram > 0) {
        ESP_LOGI(TAG, "PSRAM Usage: %" PRIu32 "%% used, %" PRIu32 "%% free", 
                 (uint32_t)((stats.total_psram - stats.free_psram) * 100 / stats.total_psram),
                 (uint32_t)(stats.free_psram * 100 / stats.total_psram));
    }
#endif
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_cpu_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Buffer to store task stats
    char *buf = heap_caps_malloc(RM_PRINT_BUFFER_SIZE, MALLOC_CAP_8BIT);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for CPU stats");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "==== CPU USAGE BY TASK ====");
    
    // Get task runtime statistics
    vTaskGetRunTimeStats(buf);
    ESP_LOGI(TAG, "Task Name\tRuntime\t\tPercentage");
    ESP_LOGI(TAG, "%s", buf);
    
    heap_caps_free(buf);
    return ESP_OK;
}

/**
 * @brief Convert task state to readable string
 * 
 * @param state FreeRTOS task state
 * @return const char* Human-readable state description
 */
static const char *task_state_to_string(eTaskState state)
{
    switch (state) {
        case eRunning:   return "Running";
        case eReady:     return "Ready";
        case eBlocked:   return "Blocked";
        case eSuspended: return "Suspended";
        case eDeleted:   return "Deleted";
        case eInvalid:   return "Invalid";
        default:         return "Unknown";
    }
}

esp_err_t resource_monitor_print_stack_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get the number of tasks
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    if (task_count == 0) {
        ESP_LOGW(TAG, "No tasks found");
        return ESP_OK;
    }
    
    // Limit task count to reasonable number to prevent excessive memory allocation
    if (task_count > RM_MAX_TASK_COUNT) {
        ESP_LOGW(TAG, "Limiting task info to %d tasks (actual: %u)", RM_MAX_TASK_COUNT, task_count);
        task_count = RM_MAX_TASK_COUNT;
    }
    
    // Allocate memory for task information
    TaskStatus_t *task_info = heap_caps_malloc(task_count * sizeof(TaskStatus_t), MALLOC_CAP_8BIT);
    if (task_info == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for task stack stats");
        return ESP_ERR_NO_MEM;
    }
    
    // Get task information
    UBaseType_t actual_task_count = uxTaskGetSystemState(task_info, task_count, NULL);
    
    if (actual_task_count == 0) {
        ESP_LOGW(TAG, "Failed to get task information");
        heap_caps_free(task_info);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "==== TASK STACK USAGE ====");
    ESP_LOGI(TAG, "%-16s %-10s %-10s %-10s %-8s %-10s", 
             "Task Name", "Priority", "State", "Stack Size", "Stack HWM", "Free Stack");
    
    for (UBaseType_t i = 0; i < actual_task_count && i < task_count; i++) {
        UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(task_info[i].xHandle);
        UBaseType_t stack_size = 0;
        bool stack_size_known = false;
        
        #if (configRECORD_STACK_HIGH_ADDRESS == 1)
        // If this config option is available, we can calculate total stack size
        // Removed the problematic code with pxStack and pxEndOfStack
        #if (portSTACK_GROWTH < 0)
        // For architectures where stack grows downwards (like ESP32)
        // Get the stack size directly from task control block if available
        // or from stack high water mark (approximate value)
        stack_size = (UBaseType_t)task_info[i].usStackHighWaterMark;
        stack_size_known = true;
        #else
        // For architectures where stack grows upwards
        // Not typical for ESP32, but included for completeness
        stack_size_known = false;
        #endif
        #endif
        
        ESP_LOGI(TAG, "%-16s %-10u %-10s %-10s %-8u %-10u", 
                 task_info[i].pcTaskName, 
                 task_info[i].uxCurrentPriority, 
                 task_state_to_string(task_info[i].eCurrentState),
                 stack_size_known ? "Known" : "Unknown",
                 stack_hwm * sizeof(StackType_t),
                 stack_size_known ? ((stack_size - stack_hwm) * sizeof(StackType_t)) : 0);
    }
    
    heap_caps_free(task_info);
    return ESP_OK;
}

esp_err_t resource_monitor_print_flash_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "==== FLASH FILESYSTEM USAGE ====");
    bool any_filesystem_found = false;
    
    // Try to get SPIFFS stats
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        any_filesystem_found = true;
        ESP_LOGI(TAG, "SPIFFS: Total: %u bytes, Used: %u bytes, Free: %u bytes (%.1f%%)",
                (unsigned int)total, (unsigned int)used, 
                (unsigned int)(total - used),
                (total > 0) ? ((total - used) * 100.0f) / total : 0.0f);
    }
    
    // Try to get FAT filesystem stats
    FATFS *fs;
    DWORD free_clusters, total_clusters, sector_size;
    
    if (f_getfree("0:", &free_clusters, &fs) == FR_OK) {
        any_filesystem_found = true;
        total_clusters = (fs->n_fatent - 2);
        sector_size = fs->csize * 512;  // Typical sector size
        
        ESP_LOGI(TAG, "FAT: Total: %u bytes, Free: %u bytes (%.1f%%)",
                (unsigned int)(total_clusters * sector_size),
                (unsigned int)(free_clusters * sector_size),
                (total_clusters > 0) ? ((free_clusters * 100.0f) / total_clusters) : 0.0f);
    }
    
    if (!any_filesystem_found) {
        ESP_LOGW(TAG, "No filesystem statistics available");
    }
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_comprehensive(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "===============================");
    ESP_LOGI(TAG, "COMPREHENSIVE RESOURCE MONITOR");
    ESP_LOGI(TAG, "===============================");
    
    esp_err_t ret;
    
    // Removed call to resource_monitor_print_sys_stats
    
    ret = resource_monitor_print_mem_stats();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to print memory stats: %s", esp_err_to_name(ret));
        // Continue with other stats
    }
    
    ret = resource_monitor_print_cpu_stats();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to print CPU stats: %s", esp_err_to_name(ret));
        // Continue with other stats
    }
    
    ret = resource_monitor_print_stack_stats();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to print stack stats: %s", esp_err_to_name(ret));
        // Continue with other stats
    }
    
    ret = resource_monitor_print_flash_stats();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to print flash stats: %s", esp_err_to_name(ret));
        // Continue with other stats
    }
    
    return ESP_OK;
}

/**
 * @brief Periodic timer callback to print resource information
 * 
 * @param timer Timer handle
 */
static void periodic_timer_callback(TimerHandle_t timer)
{
    resource_monitor_print_comprehensive();
}

esp_err_t resource_monitor_start_periodic(uint32_t interval_ms)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate interval
    if (interval_ms < RM_MIN_INTERVAL_MS) {
        ESP_LOGW(TAG, "Interval too small, using minimum: %u ms", RM_MIN_INTERVAL_MS);
        interval_ms = RM_MIN_INTERVAL_MS;
    }
    
    if (s_rm_ctx.periodic_running) {
        ESP_LOGD(TAG, "Periodic monitoring already running, updating interval");
        
        // Update timer period
        if (xTimerChangePeriod(s_rm_ctx.periodic_timer, pdMS_TO_TICKS(interval_ms), portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to update periodic timer interval");
            return ESP_FAIL;
        }
        
        ESP_LOGI(TAG, "Updated periodic resource monitoring interval to %" PRIu32 "ms", interval_ms);
        return ESP_OK;
    }
    
    // Create timer if it doesn't exist
    if (s_rm_ctx.periodic_timer == NULL) {
        s_rm_ctx.periodic_timer = xTimerCreate(
            "rm_timer",
            pdMS_TO_TICKS(interval_ms),
            pdTRUE,  // Auto reload
            NULL,
            periodic_timer_callback
        );
        
        if (s_rm_ctx.periodic_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create periodic timer");
            return ESP_ERR_NO_MEM;
        }
    } else {
        // Update timer period
        if (xTimerChangePeriod(s_rm_ctx.periodic_timer, pdMS_TO_TICKS(interval_ms), portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to update periodic timer interval");
            return ESP_FAIL;
        }
    }
    
    // Start timer
    if (xTimerStart(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start periodic timer");
        return ESP_FAIL;
    }
    
    s_rm_ctx.periodic_running = true;
    ESP_LOGI(TAG, "Started periodic resource monitoring (interval: %" PRIu32 "ms)", interval_ms);
    
    return ESP_OK;
}

esp_err_t resource_monitor_stop_periodic(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_OK; // Not initialized
    }
    
    if (!s_rm_ctx.periodic_running) {
        return ESP_OK; // Not running
    }
    
    if (s_rm_ctx.periodic_timer == NULL) {
        ESP_LOGW(TAG, "Periodic timer not found but marked as running");
        s_rm_ctx.periodic_running = false;
        return ESP_OK;
    }
    
    if (xTimerStop(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to stop periodic timer");
        return ESP_FAIL;
    }
    
    s_rm_ctx.periodic_running = false;
    ESP_LOGI(TAG, "Stopped periodic resource monitoring");
    
    return ESP_OK;
}
