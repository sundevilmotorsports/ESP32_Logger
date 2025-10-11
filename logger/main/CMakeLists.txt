idf_component_register(SRCS "sdcard.c" "gnss.c" "adc.c" "ina260.c" "logger.c" "main.c"
                            "dtc.c"  # Add this line to include dtc.c
                            "can.c"
                            "sdcard.c"
                    INCLUDE_DIRS ".")