idf_component_register(SRCS "gnss.c" "adc.c" "ina260.c" "logger.c" "main.c"
                            "dtc.c"  # Add this line to include dtc.c
                            "can.c"
                    INCLUDE_DIRS ".")