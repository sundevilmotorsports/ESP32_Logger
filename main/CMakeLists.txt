idf_component_register(SRCS "adc_driver.cpp" "main.cpp" "logger.cpp"
                    INCLUDE_DIRS "." "include"
                    REQUIRES ModuleCore driver esp_timer fatfs)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++23)
