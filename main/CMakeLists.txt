idf_component_register(SRCS "hello_world_main.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES apds9960
                    INCLUDE_DIRS "")
