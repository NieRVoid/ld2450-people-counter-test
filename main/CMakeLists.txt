idf_component_register(
    SRCS "people_counter_example.c"
    INCLUDE_DIRS ""
    REQUIRES nvs_flash driver esp_timer console ld2450_driver people_counter
    PRIV_REQUIRES spi_flash
)
