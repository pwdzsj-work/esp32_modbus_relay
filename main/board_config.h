#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#define BOARD_RELAY_COUNT              4
#define BOARD_INPUT_COUNT              4
#define BOARD_ANALOG_COUNT             4

/* Relay optocoupler + ULN2003 chain is active-low at the ESP32 pins. */
#define BOARD_RELAY_ACTIVE_LEVEL       0
#define BOARD_RELAY_INACTIVE_LEVEL     1
#define BOARD_RELAY1_GPIO              GPIO_NUM_33
#define BOARD_RELAY2_GPIO              GPIO_NUM_25
#define BOARD_RELAY3_GPIO              GPIO_NUM_26
#define BOARD_RELAY4_GPIO              GPIO_NUM_27

/* NQ inputs are PC817 collectors with 10 k pull-ups: asserted input reads low. */
#define BOARD_NQ_ACTIVE_LEVEL          0
#define BOARD_NQ1_GPIO                 GPIO_NUM_18
#define BOARD_NQ2_GPIO                 GPIO_NUM_2
#define BOARD_NQ3_GPIO                 GPIO_NUM_32
#define BOARD_NQ4_GPIO                 GPIO_NUM_23

#define BOARD_I2C_PORT                 I2C_NUM_0
#define BOARD_I2C_SDA_GPIO             GPIO_NUM_21
#define BOARD_I2C_SCL_GPIO             GPIO_NUM_22
#define BOARD_I2C_FREQ_HZ              100000
#define BOARD_ADS1115_ADDR             0x48

#define BOARD_RS485_UART               UART_NUM_2
#define BOARD_RS485_TX_GPIO            GPIO_NUM_13
#define BOARD_RS485_RX_GPIO            GPIO_NUM_14
#define BOARD_RS485_DIR_GPIO           GPIO_NUM_19
#define BOARD_RS485_BAUD               9600
#define BOARD_RS485_DIR_RX_LEVEL       0
#define BOARD_RS485_DIR_TX_LEVEL       1
#define BOARD_MODBUS_SLAVE_ADDR        1
