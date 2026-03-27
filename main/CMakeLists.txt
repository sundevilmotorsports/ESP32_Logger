idf_component_register(SRCS "main.cpp" "logger.cpp" "adc_device.cpp" "ext_adc_oneshot.cpp" "ext_spi_adc.cpp"
                    INCLUDE_DIRS "." "include"
                    REQUIRES ModuleCore driver esp_timer fatfs)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++23)
