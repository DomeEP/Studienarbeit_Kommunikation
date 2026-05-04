/**
 * @file app_config.h
 * @brief Central configuration for the T3200 communication firmware
 *
 * All adjustable parameters in one place. When extending the system
 * (new registers, additional slaves, different baud rates), only this
 * file needs to be modified.
 *
 * Hardware overview (from schematic):
 *
 *   WeActStudio STM32G474 CoreBoard
 *
 *   On-board:
 *     PC13 = BTN_intern  (user button, active-LOW)
 *     PC6  = LED_intern  (on-board LED)
 *
 *   External (master PCB):
 *     PB6  = LED1, PB7 = LED2, PB9 = BTN
 *
 *   RS485 (ADM2587E):
 *     PA1 = USART2_DE (driver enable)
 *     PA2 = USART2_TX, PA3 = USART2_RX
 *
 *   I2C (expansion):
 *     PA8 = I2C2_SDA, PA9 = I2C2_SCL
 *
 *   Power:
 *     USB-C  -> 5V/3.3V for STM32 + ADM2587E logic side
 *     J1 12V -> TBA1-1211 DC-DC -> isolated 5V for ADM2587E bus side
 *     NOTE: RS485 bus is dead without 12V on J1
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Modbus slave address (only relevant in slave mode, range 1-247) */
#define APP_SLAVE_ID              1

/* RS485 baud rate (must match on all bus participants) */
#define APP_BAUDRATE              115200

/* Master response timeout in ms */
#define APP_RESPONSE_TIMEOUT_MS   1000

/* Total register map size */
#define APP_REGISTER_MAP_SIZE     20

/* LED heartbeat interval in ms */
#define APP_HEARTBEAT_INTERVAL_MS 500

/* Button debounce delay in ms */
#define APP_DEBOUNCE_DELAY_MS     50

/*
 * Extension notes:
 *
 * 1. Add a new slave:
 *    - Change APP_SLAVE_ID (each slave needs a unique address)
 *    - Update master code to target the new ID
 *
 * 2. Add a new register:
 *    - Increase APP_REGISTER_MAP_SIZE
 *    - Define REG_xxx in modbus_rtu.h
 *    - Handle the new register in the slave write callback
 *
 * 3. Send sensor data:
 *    - Slave: write value to register_map[REG_xxx]
 *    - Master: read it via Modbus_Master_Request(FC03, REG_xxx, 1)
 */

#endif /* APP_CONFIG_H */
