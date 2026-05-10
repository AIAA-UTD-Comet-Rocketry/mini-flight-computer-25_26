idf_component_register(SRCS "canaerospace.cpp"
                            "can_telemetry.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_twai
                                FlightFSM
                                main
                       PRIV_REQUIRES driver freertos esp_timer)
