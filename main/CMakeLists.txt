idf_component_register(SRCS main.cpp
                        MqttClient.cpp
                    PRIV_REQUIRES spi_flash
                    REQUIRES
							mqtt
							apds9960
                             ldradar
                             jsonwrapper
                            button
                            wifimanager
                            nvsstoragemanager
                    INCLUDE_DIRS "")
