/**
 * @file modbus_rtu.h
 * @brief Modbus RTU protocol library for STM32G4 (T3200 grid stabilization)
 * @version 3.0
 * @date 2026-02-21
 *
 * Requires: UART in RS485 mode (hardware DE), isolated transceiver (ADM2587E),
 * basic timer (e.g. TIM6) for T3.5 frame detection.
 * Timer auto-calibrates to the correct T3.5 period on init.
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================================================== */
/*                                 CONFIGURATION                                      */
/* ================================================================================== */

#define MODBUS_ENABLE_MASTER  1
#define MODBUS_ENABLE_SLAVE   1

#define MODBUS_MAX_ADU_SIZE   256

#define MODBUS_RESPONSE_TIMEOUT_MS 1000
#define MODBUS_TURNAROUND_DELAY_MS 5

/* ================================================================================== */
/*                           SYSTEM REGISTERS                                         */
/* ================================================================================== */

#define MODE_STANDBY   0
#define MODE_CHARGE    1
#define MODE_DISCHARGE 2

/* Slave -> Master (FC03) */
#define REG_WUNSCH     0
#define REG_EMERGENCY  1
#define REG_NODE_TYPE  2   /* 1 = Inverter, 2 = DC/DC */

/* Master -> Slave (FC06) */
#define REG_ERLAUBNIS  10
#define REG_SAFETY_STOP 11

/* ================================================================================== */
/*                                 DEFINITIONS                                        */
/* ================================================================================== */

#define MB_FC_READ_HOLDING_REGISTERS   0x03
#define MB_FC_WRITE_SINGLE_REGISTER    0x06
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10
#define MB_FC_ERROR_OFFSET             0x80

#define MB_EX_ILLEGAL_FUNCTION       0x01
#define MB_EX_ILLEGAL_DATA_ADDR      0x02
#define MB_EX_ILLEGAL_DATA_VALUE     0x03
#define MB_EX_SERVER_DEVICE_FAILURE  0x04

typedef enum {
    MB_STATE_IDLE,
    MB_STATE_RX,
    MB_STATE_PROCESSING,
    MB_STATE_TX,
    MB_STATE_WAIT_RESPONSE
} Modbus_State_t;

typedef enum {
    MB_ERROR_NONE = 0,
    MB_ERROR_TIMEOUT,
    MB_ERROR_CRC,
    MB_ERROR_EXCEPTION,
    MB_ERROR_TRANSMIT,
    MB_ERROR_SIZE
} Modbus_Error_t;

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t bus_errors;
} Modbus_Stats_t;

typedef struct {
    UART_HandleTypeDef *huart;
    TIM_HandleTypeDef  *htim;

    uint8_t  slave_id;

    uint8_t  rx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t rx_index;
    volatile bool frame_complete;

    uint8_t  tx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t tx_length;

    volatile Modbus_State_t state;
    uint32_t last_activity_timestamp;

    uint8_t  pending_func_code;

    Modbus_Stats_t stats;

    bool (*validate_addr_cb)(uint16_t addr);
    void (*write_reg_cb)(uint16_t addr, uint16_t val);
    void (*master_complete_cb)(void);
    void (*error_cb)(Modbus_Error_t error);

} Modbus_Handle_t;

/* ================================================================================== */
/*                                    API                                             */
/* ================================================================================== */

void Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id);

HAL_StatusTypeDef Modbus_Master_Request(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_val);
HAL_StatusTypeDef Modbus_Master_WriteMultiple(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint16_t start_addr, uint16_t reg_count, uint16_t *data);
Modbus_Error_t    Modbus_Master_Process(Modbus_Handle_t *hmodbus);

void Modbus_Slave_Listen(Modbus_Handle_t *hmodbus, uint16_t *register_map, uint16_t map_size);

/* ================================================================================== */
/*                             INTERRUPT HANDLERS                                     */
/* ================================================================================== */

void Modbus_IRQHandler_RxCplt(Modbus_Handle_t *hmodbus);
void Modbus_IRQHandler_Timeout(Modbus_Handle_t *hmodbus);
void Modbus_IRQHandler_Error(Modbus_Handle_t *hmodbus);

#endif /* MODBUS_RTU_H */
