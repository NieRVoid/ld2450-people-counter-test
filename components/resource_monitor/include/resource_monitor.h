/**
 * @file resource_monitor.h
 * @brief Simple resource monitoring component for ESP32
 * 
 * This component provides functions to monitor memory, task, and 
 * filesystem resources on ESP32 devices.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Memory statistics structure
 */
typedef struct {
    size_t total_heap;          /*!< Total heap size */
    size_t free_heap;           /*!< Current free heap */
    size_t min_free_heap;       /*!< Minimum free heap since boot */
    size_t max_alloc_heap;      /*!< Largest allocatable block */
#ifdef CONFIG_SPIRAM
    size_t total_psram;         /*!< Total PSRAM size */
    size_t free_psram;          /*!< Current free PSRAM */
    size_t min_free_psram;      /*!< Minimum free PSRAM since boot */
    size_t max_alloc_psram;     /*!< Largest allocatable PSRAM block */
#endif
} resource_monitor_mem_stats_t;

/**
 * @brief Initialize the resource monitor
 *
 * This must be called before using other resource monitor functions.
 * The function is idempotent and can be called multiple times safely.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_init(void);

/**
 * @brief Deinitialize the resource monitor
 * 
 * Stops any running periodic monitoring and releases resources.
 * The function is idempotent and can be called multiple times safely.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_deinit(void);

/**
 * @brief Get memory usage statistics
 * 
 * @param[out] stats Pointer to store memory statistics
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if stats is NULL,
 *                   ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t resource_monitor_get_mem_stats(resource_monitor_mem_stats_t *stats);

/**
 * @brief Print memory usage statistics
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t resource_monitor_print_mem_stats(void);

/**
 * @brief Print CPU usage by task
 * 
 * Prints CPU usage information for all tasks to the log.
 * Requires configGENERATE_RUN_TIME_STATS to be enabled.
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
 *                   ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t resource_monitor_print_cpu_stats(void);

/**
 * @brief Print task stack usage information
 * 
 * Prints stack usage for all tasks to the log.
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
 *                   ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t resource_monitor_print_stack_stats(void);

/**
 * @brief Print flash filesystem usage statistics
 * 
 * Attempts to get statistics for both SPIFFS and FAT filesystems.
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t resource_monitor_print_flash_stats(void);

/**
 * @brief Print comprehensive resource statistics
 * 
 * Prints all available resource statistics including memory, CPU usage,
 * task stack usage, and flash usage.
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t resource_monitor_print_comprehensive(void);

/**
 * @brief Start periodic resource usage printing
 * 
 * @param interval_ms Interval between prints in milliseconds (minimum 100ms)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
 *                   ESP_ERR_INVALID_ARG if interval is too small,
 *                   ESP_ERR_NO_MEM if timer creation fails
 */
esp_err_t resource_monitor_start_periodic(uint32_t interval_ms);

/**
 * @brief Stop periodic resource usage printing
 * 
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
 *                   ESP_FAIL if timer fails to stop
 */
esp_err_t resource_monitor_stop_periodic(void);

#ifdef __cplusplus
}
#endif
