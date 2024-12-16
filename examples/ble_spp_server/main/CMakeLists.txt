idf_component_register(
    SRCS 
        "ble_spp_server_demo.c"
        "ui.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver 
        esp_driver_gpio
        bt
        m5stack_core_2
        nvs_flash
)
