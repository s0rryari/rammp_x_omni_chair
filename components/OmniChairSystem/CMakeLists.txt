idf_component_register(SRCS "src/OmniChair_system.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES base_component logger task rtps cdr esp32-p4-eth ethernet
                                esp_netif rammp_rtps_messages basicmicro esp_driver_uart)
