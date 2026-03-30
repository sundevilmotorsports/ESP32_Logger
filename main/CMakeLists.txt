idf_component_register(SRCS "main.cpp" "logger.cpp" "gnss.cpp"
                    INCLUDE_DIRS "." "include"
                    REQUIRES ModuleCore esp_adc esp_timer fatfs)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++23)
