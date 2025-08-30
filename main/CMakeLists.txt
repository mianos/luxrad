idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES spi_flash
                    REQUIRES apds9960
                             ldradar
                             jsonwrapper
                            button
                            wifimanager
                            nvsstoragemanager
                    INCLUDE_DIRS "")
