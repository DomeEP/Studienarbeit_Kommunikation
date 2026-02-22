/**
 * @file modbus_rtu.h
 * @brief Professional Modbus RTU Library for STM32G474 (Grid Stabilization System)
 * @author Senior Embedded Systems Engineer
 * @version 2.0
 * @date 2026-02-21
 * 
 * @note Hardware Requirements:
 * - STM32G474 with ADM2587E Isolated Transceiver.
 * - UART configured in RS485 Mode (Hardware DE signal).
 * - Basic Timer (e.g., TIM6) for T3.5 character timeout.
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================================================== */
/*                                 CONFIGURATION                                      */
/* ================================================================================== */

// Enable/Disable Modes to save Flash/RAM
#define MODBUS_ENABLE_MASTER  1
#define MODBUS_ENABLE_SLAVE   1

// Buffer Size (Adjust based on max expected frame size)
#define MODBUS_MAX_ADU_SIZE   256

// Timeout Settings
#define MODBUS_RESPONSE_TIMEOUT_MS 1000  // Master waits 1s for Slave response

/* ================================================================================== */
/*                                 REGISTER MAP                                       */
/* ================================================================================== */
// Slave Register Map (Holding Registers - 40001 offset)
#define REG_ADDR_CONTROL_MODE  0x0000 // R/W: 0=Idle, 1=Charge, 2=Discharge
#define REG_ADDR_CELL_VOLTAGE  0x0001 // R: Unit 10mV
#define REG_ADDR_CURRENT       0x0002 // R: Unit 10mA (Offset +30000)
#define REG_ADDR_SOC           0x0003 // R: Unit 0.1%

/* ================================================================================== */
/*                                 DEFINITIONS                                        */
/* ================================================================================== */

// Modbus Function Codes
#define MB_FC_READ_HOLDING_REGISTERS 0x03
#define MB_FC_WRITE_SINGLE_REGISTER  0x06
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10 // Optional
#define MB_FC_ERROR_OFFSET           0x80

// Exception Codes
#define MB_EX_ILLEGAL_FUNCTION       0x01
#define MB_EX_ILLEGAL_DATA_ADDR      0x02
#define MB_EX_ILLEGAL_DATA_VALUE     0x03
#define MB_EX_SERVER_DEVICE_FAILURE  0x04

typedef enum {
    MB_STATE_IDLE,
    MB_STATE_RX,            // Receiving data
    MB_STATE_PROCESSING,    // Parsing frame (Slave) or Handling Response (Master)
    MB_STATE_TX,            // Transmitting
    MB_STATE_WAIT_RESPONSE  // Master only
} Modbus_State_t;

typedef enum {
    MB_ERROR_NONE = 0,
    MB_ERROR_TIMEOUT,
    MB_ERROR_CRC,
    MB_ERROR_EXCEPTION,
    MB_ERROR_TRANSMIT
} Modbus_Error_t;

/**
 * @brief Modbus Handle Structure
 */
typedef struct {
    // Hardware Handles
    UART_HandleTypeDef *huart;
    TIM_HandleTypeDef  *htim;
    
    // Configuration
    uint8_t  slave_id;       // Local ID (Slave) or Target ID (Master context)
    
    // Buffers
    uint8_t  rx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t rx_index;
    volatile bool frame_complete; // Set by T3.5 Timer ISR
    
    uint8_t  tx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t tx_length;

    // State Machine
    volatile Modbus_State_t state;
    uint32_t last_activity_timestamp;
    
    // Master Specific
    uint8_t  pending_func_code; // To verify response matches request

    // Callbacks (Slave Mode)
    void (*write_reg_cb)(uint16_t addr, uint16_t val);
    
} Modbus_Handle_t;

/* ================================================================================== */
/*                                    API                                             */
/* ================================================================================== */

/**
 * @brief Initialize the Modbus Library
 */
void Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id);

/**
 * @brief Master: Send a Request (Non-blocking)
 * @param slave_id Target Slave Address
 * @param func_code Function Code (0x03, 0x06)
 * @param reg_addr Start Register Address
 * @param reg_val  Value to write (for FC06) or Number of registers (for FC03)
 */
HAL_StatusTypeDef Modbus_Master_Request(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_val);

/**
 * @brief Slave: Listen and Process Requests (Call in Main Loop)
 * @details Checks for complete frames, validates CRC, and updates registers.
 *          Maps internal variables to the defined Register Map.
 * @param data_store Pointer to the struct/array holding the actual system data (Voltage, Current, etc.)
 */
void Modbus_Slave_Listen(Modbus_Handle_t *hmodbus, uint16_t *register_map, uint16_t map_size);

/**
 * @brief Master: Process Responses (Call in Main Loop)
 * @return MB_ERROR_NONE if successful or idle
 */
Modbus_Error_t Modbus_Master_Process(Modbus_Handle_t *hmodbus);

/* ================================================================================== */
/*                             INTERRUPT HANDLERS                                     */
/* ================================================================================== */
// Place these in stm32g4xx_it.c or callback overrides

void Modbus_IRQHandler_RxCplt(Modbus_Handle_t *hmodbus);
void Modbus_IRQHandler_Timeout(Modbus_Handle_t *hmodbus);

#endif /* MODBUS_RTU_H */