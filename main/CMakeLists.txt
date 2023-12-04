idf_component_register(SRCS "BitcoinTicker.c" "fontx.c" "st7735s.c"
                    INCLUDE_DIRS ".")

                    spiffs_create_partition_image(storage font FLASH_IN_PROJECT)