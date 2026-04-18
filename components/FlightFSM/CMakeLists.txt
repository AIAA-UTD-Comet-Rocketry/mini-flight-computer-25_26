idf_component_register(SRCS "FlightStateAdapter.c" "FlightFSM.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "esp_timer" "freertos"
                    WHOLE_ARCHIVE)
