# components/ld2450-driver/CMakeLists.txt

idf_component_register(
    SRCS
        "src/ld2450.c"
        "src/ld2450_config.c"
        "src/ld2450_parser.c"
        "src/ld2450_circular_buffer.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        driver
        log
        freertos
        esp_event
        esp_common
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
