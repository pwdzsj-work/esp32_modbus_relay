#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#define BOARD_RELAY_COUNT              4
#define BOARD_INPUT_COUNT              8
#define BOARD_ANALOG_COUNT             4

/*
 * RELAY1..4 drive the PC817 LEDs through 1.5 k resistors.
 * A high ESP32 output enables the optocoupler and the ULN2003 channel.
 */
#define BOARD_RELAY_ACTIVE_LEVEL       1
#define BOARD_RELAY_INACTIVE_LEVEL     0
#define BOARD_RELAY1_GPIO              GPIO_NUM_25
#define BOARD_RELAY2_GPIO              GPIO_NUM_26
#define BOARD_RELAY3_GPIO              GPIO_NUM_27
#define BOARD_RELAY4_GPIO              GPIO_NUM_33

/* NQ inputs are PC817 collectors with 10 k pull-ups: asserted input reads low. */
#define BOARD_NQ_ACTIVE_LEVEL          0
#define BOARD_NQ1_GPIO                 GPIO_NUM_18
#define BOARD_NQ2_GPIO                 GPIO_NUM_2
#define BOARD_NQ3_GPIO                 GPIO_NUM_32
#define BOARD_NQ4_GPIO                 GPIO_NUM_23
#define BOARD_NQ_SIGNAL_GPIO_MASK      ((1ULL << BOARD_NQ1_GPIO) | \
                                        (1ULL << BOARD_NQ2_GPIO) | \
                                        (1ULL << BOARD_NQ3_GPIO) | \
                                        (1ULL << BOARD_NQ4_GPIO))
/* Relay feedback inputs from U16..U19. */
#define BOARD_NQ5_GPIO                 GPIO_NUM_34
#define BOARD_NQ6_GPIO                 GPIO_NUM_35
#define BOARD_NQ7_GPIO                 GPIO_NUM_36
#define BOARD_NQ8_GPIO                 GPIO_NUM_39
#define BOARD_RELAY_FEEDBACK_FIRST_INPUT 4
#define BOARD_NQ_FEEDBACK_GPIO_MASK    ((1ULL << BOARD_NQ5_GPIO) | \
                                        (1ULL << BOARD_NQ6_GPIO) | \
                                        (1ULL << BOARD_NQ7_GPIO) | \
                                        (1ULL << BOARD_NQ8_GPIO))

#define BOARD_I2C_PORT                 I2C_NUM_0
#define BOARD_I2C_SDA_GPIO             GPIO_NUM_21
#define BOARD_I2C_SCL_GPIO             GPIO_NUM_22
#define BOARD_I2C_FREQ_HZ              100000
#define BOARD_ADS1115_ADDR             0x48
#define BOARD_RV3028_ADDR              0x52

/*
 * Ground SW3 while powering up/resetting to enable Wi-Fi configuration and
 * the local web control page.
 */
#define BOARD_WEB_CONFIG_GPIO          GPIO_NUM_15
#define BOARD_WEB_CONFIG_ACTIVE_LEVEL  0
#define BOARD_WEB_AP_SSID              "Relay-WebConfig"
#define BOARD_WEB_AP_PASSWORD          "12345678"
/* Required by the Web OTA upload endpoint. Change before deployment. */
#define BOARD_WEB_OTA_PASSWORD         "relay-ota-2026"

#define BOARD_RS485_UART               UART_NUM_2
/* Schematic nets: TXD2=IO13, RXD2=IO14, 485C=IO19. */
#define BOARD_RS485_TX_GPIO            GPIO_NUM_13
#define BOARD_RS485_RX_GPIO            GPIO_NUM_14
#define BOARD_RS485_DIR_GPIO           GPIO_NUM_19
#define BOARD_RS485_BAUD               9600
/*
 * 485C drives the optocoupler LED cathode, so it is active-low:
 * low enables ADM483 transmission; high enables reception.
 */
#define BOARD_RS485_DIR_RX_LEVEL       1
#define BOARD_RS485_DIR_TX_LEVEL       0
#define BOARD_MODBUS_SLAVE_ADDR        1

/*
 * Returned by Modbus function 0x11 (Report Server ID).  A Qt host can poll
 * slave addresses 1..247 with this function to discover boards through a
 * transparent LoRa link.
 */
#define BOARD_MODBUS_SERVER_ID         BOARD_MODBUS_SLAVE_ADDR
/* Stable product SKU reported by both discovery protocols. */
#define BOARD_PRODUCT_SKU              "ESP32-RELAY-4"
#define BOARD_DEVICE_MODEL             BOARD_PRODUCT_SKU
