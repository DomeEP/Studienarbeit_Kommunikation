/**
 * @file modbus_rtu.h
 * @brief Professional Modbus RTU Library for STM32G474 (Grid Stabilization System)
 * @author Senior Embedded Systems Engineer
 * @version 3.0
 * @date 2026-02-21
 * 
 * @note Hardware Requirements:
 * - STM32G474 with ADM2587E Isolated Transceiver.
 * - UART configured in RS485 Mode (Hardware DE signal).
 * - Basic Timer (e.g., TIM6) for T3.5 character timeout.
 *
 * @details
 * This library implements the Modbus RTU protocol. It uses a hardware timer to detect
 * the end of a frame (silence of >3.5 character times).
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

// Buffer Size (Must be large enough for the largest expected Modbus frame)
#define MODBUS_MAX_ADU_SIZE   256

// Timeout Settings (in Milliseconds)
#define MODBUS_RESPONSE_TIMEOUT_MS 1000  // Master waits 1s for Slave response

/* ================================================================================== */
/*                                 DEFINITIONS                                        */
/* ================================================================================== */

// Modbus Function Codes
#define MB_FC_READ_HOLDING_REGISTERS   0x03
#define MB_FC_WRITE_SINGLE_REGISTER    0x06
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10 // New: Bulk Write
#define MB_FC_ERROR_OFFSET             0x80

// Exception Codes
#define MB_EX_ILLEGAL_FUNCTION       0x01
#define MB_EX_ILLEGAL_DATA_ADDR      0x02
#define MB_EX_ILLEGAL_DATA_VALUE     0x03
#define MB_EX_SERVER_DEVICE_FAILURE  0x04

// Modbus Protocol States
typedef enum {
    MB_STATE_IDLE,          // Waiting for start of frame
    MB_STATE_RX,            // Receiving bytes (Timer running)
    MB_STATE_PROCESSING,    // Frame received, processing data
    MB_STATE_TX,            // Transmitting response
    MB_STATE_WAIT_RESPONSE  // Master only: Waiting for Slave reply
} Modbus_State_t;

// Error Codes for internal handling
typedef enum {
    MB_ERROR_NONE = 0,
    MB_ERROR_TIMEOUT,       // Response timeout
    MB_ERROR_CRC,           // Checksum mismatch
    MB_ERROR_EXCEPTION,     // Slave returned an error exception
    MB_ERROR_TRANSMIT,      // UART Transmission error
    MB_ERROR_SIZE           // Frame too short or buffer overflow
} Modbus_Error_t;

/**
 * @brief Diagnostic Statistics
 * Useful for debugging RS485 bus health.
 */
typedef struct {
    uint32_t rx_frames;     // Total valid frames received
    uint32_t tx_frames;     // Total frames sent
    uint32_t crc_errors;    // Corrupted frames received
    uint32_t timeouts;      // Response timeouts (Master only)
    uint32_t bus_errors;    // UART hardware errors (Noise, etc.)
} Modbus_Stats_t;

/**
 * @brief Modbus Handle Structure
 * Contains all state variables for a single Modbus instance.
 */
typedef struct {
    // --- Hardware Handles ---
    UART_HandleTypeDef *huart;      // Pointer to STM32 UART Handle
    TIM_HandleTypeDef  *htim;       // Pointer to STM32 Timer Handle (for T3.5)
    
    // --- Configuration ---
    uint8_t  slave_id;              // My Address (Slave Mode) or Target Address (Master Mode temporary)
    
    // --- Buffers ---
    uint8_t  rx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t rx_index;
    volatile bool frame_complete;   // Flag set by Timer ISR when frame ends
    
    uint8_t  tx_buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t tx_length;

    // --- State Machine ---
    volatile Modbus_State_t state;
    uint32_t last_activity_timestamp;
    
    // --- Master Specific Context ---
    uint8_t  pending_func_code;     // Expected Function Code in response
    
    // --- Diagnostics ---
    Modbus_Stats_t stats;

    // --- User Callbacks (Optional) ---
    // Called to check if an address is valid (Slave Mode)
    // Return true if valid, false if invalid (sends Exception 02)
    bool (*validate_addr_cb)(uint16_t addr);

    // Called when a valid Register Write occurs (Slave Mode)
    void (*write_reg_cb)(uint16_t addr, uint16_t val); 
    
    // Called when a Master request completes successfully
    void (*master_complete_cb)(void);

    // Called on Error
    void (*error_cb)(Modbus_Error_t error);
    
} Modbus_Handle_t;

/* ================================================================================== */
/*                                    API                                             */
/* ================================================================================== */

/**
 * @brief Initialize the Modbus Library
 * @param hmodbus Pointer to your Modbus handle
 * @param huart   STM32 UART handle (configured for RS485)
 * @param htim    STM32 Timer handle (configured for T3.5 timeout)
 * @param slave_id The Modbus Address of this device (1-247)
 */
void Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id);

/**
 * @brief Master: Send a Request (Non-blocking)
 * @param slave_id Target Slave Address
 * @param func_code Function Code (0x03, 0x06, 0x10)
 * @param reg_addr Start Register Address
 * @param reg_val  Value (for FC06) or Count (for FC03)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Modbus_Master_Request(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_val);

/**
 * @brief Master: Send Write Multiple Registers (FC16)
 * @param slave_id Target Slave Address
 * @param start_addr Starting Register Address
 * @param reg_count Number of registers to write
 * @param data Pointer to the data array (uint16_t[])
 */
HAL_StatusTypeDef Modbus_Master_WriteMultiple(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint16_t start_addr, uint16_t reg_count, uint16_t *data);

/**
 * @brief Master: Process Responses (Call in Main Loop)
 * @return MB_ERROR_NONE if successful or idle
 */
Modbus_Error_t Modbus_Master_Process(Modbus_Handle_t *hmodbus);

/**
 * @brief Slave: Listen and Process Requests (Call in Main Loop)
 * @param register_map Pointer to the array holding your system data
 * @param map_size Size of the register map array
 */
void Modbus_Slave_Listen(Modbus_Handle_t *hmodbus, uint16_t *register_map, uint16_t map_size);

/* ================================================================================== */
/*                             INTERRUPT HANDLERS                                     */
/* ================================================================================== */
// CRITICAL: Call these from stm32g4xx_it.c

/**
 * @brief Call inside HAL_UART_RxCpltCallback (or USARTx_IRQHandler)
 */
void Modbus_IRQHandler_RxCplt(Modbus_Handle_t *hmodbus);

/**
 * @brief Call inside HAL_TIM_PeriodElapsedCallback (or TIMx_IRQHandler)
 */
void Modbus_IRQHandler_Timeout(Modbus_Handle_t *hmodbus);

#endif /* MODBUS_RTU_H */
