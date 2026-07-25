idf_component_register(
    SRCS
        "app_main.c"
        "ads1115.c"
        "analog_input.c"
        "digital_input.c"
        "modbus_slave.c"
        "relay_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
