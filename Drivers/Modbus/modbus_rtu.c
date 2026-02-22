/**
 * @file modbus_rtu.c
 * @brief Professional Modbus RTU Implementation
 * @author Senior Embedded Systems Engineer
 * @date 2026-02-21
 *
 * Implements Modbus RTU protocol for STM32G474.
 * Features:
 * - Robust CRC16 implementation.
 * - Non-blocking State Machine.
 * - Hardware RS485 flow control (DE) via UART configuration.
 * - T3.5 character timeout via TIM6 (or similar basic timer).
 */

#include "modbus_rtu.h"
#include <string.h>

/* ================================================================================== */
/*                                 PRIVATE PROTOTYPES                                 */
/* ================================================================================== */

static uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);
static void Modbus_Send(Modbus_Handle_t *hmodbus, uint16_t length);
static void Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t func, uint8_t exc);

/* ================================================================================== */
/*                                 CRC16 LOOKUP TABLE                                 */
/* ================================================================================== */
// Standard Modbus CRC16 Table
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741,
    0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341,
    0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40,
    0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141,
    0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41,
    0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741,
    0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941,
    0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040
};

/**
 * @brief Calculate CRC16 for Modbus
 */
static uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length) {
    uint8_t temp;
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        temp = buffer[i] ^ (uint8_t)(crc & 0xFF);
        crc = (crc >> 8) ^ crc16_table[temp];
    }
    return crc;
}

/* ================================================================================== */
/*                                    CORE FUNCTIONS                                  */
/* ================================================================================== */

void Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id) {
    hmodbus->huart = huart;
    hmodbus->htim  = htim;
    hmodbus->slave_id = slave_id;
    hmodbus->rx_index = 0;
    hmodbus->state = MB_STATE_IDLE;
    hmodbus->frame_complete = false;

    // Start Listening (Interrupt mode, 1 byte at a time to catch every char)
    HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[0], 1);
}

static void Modbus_Send(Modbus_Handle_t *hmodbus, uint16_t length) {
    // Hardware RS485 DE handles direction automatically via UART driver
    HAL_UART_Transmit(hmodbus->huart, hmodbus->tx_buffer, length, 100);
}

/* ================================================================================== */
/*                                    MASTER IMPLEMENTATION                           */
/* ================================================================================== */
#ifdef MODBUS_ENABLE_MASTER

HAL_StatusTypeDef Modbus_Master_Request(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_val) {
    if (hmodbus->state != MB_STATE_IDLE && hmodbus->state != MB_STATE_RX) {
        return HAL_BUSY; // Communication in progress
    }

    hmodbus->tx_buffer[0] = slave_id;
    hmodbus->tx_buffer[1] = func_code;
    hmodbus->tx_buffer[2] = (reg_addr >> 8) & 0xFF;
    hmodbus->tx_buffer[3] = reg_addr & 0xFF;
    hmodbus->tx_buffer[4] = (reg_val >> 8) & 0xFF;
    hmodbus->tx_buffer[5] = reg_val & 0xFF;

    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, 6);
    hmodbus->tx_buffer[6] = crc & 0xFF;
    hmodbus->tx_buffer[7] = (crc >> 8) & 0xFF;
    hmodbus->tx_length = 8;

    hmodbus->pending_func_code = func_code;
    hmodbus->state = MB_STATE_WAIT_RESPONSE;
    hmodbus->last_activity_timestamp = HAL_GetTick();

    // Clear RX buffer for response
    hmodbus->rx_index = 0;
    hmodbus->frame_complete = false;
    
    // Start T3.5 timer logic is irrelevant for TX, but we need to ensure RX is ready
    HAL_UART_Receive_IT(hmodbus->huart, hmodbus->rx_buffer, 1);
    
    Modbus_Send(hmodbus, hmodbus->tx_length);
    
    return HAL_OK;
}

Modbus_Error_t Modbus_Master_Process(Modbus_Handle_t *hmodbus) {
    if (hmodbus->state == MB_STATE_WAIT_RESPONSE) {
        // Check Timeout
        if ((HAL_GetTick() - hmodbus->last_activity_timestamp) > MODBUS_RESPONSE_TIMEOUT_MS) {
            hmodbus->state = MB_STATE_IDLE;
            return MB_ERROR_TIMEOUT;
        }

        if (hmodbus->frame_complete) {
            hmodbus->frame_complete = false;
            
            // Validate CRC
            if (hmodbus->rx_index < 4) return MB_ERROR_TRANSMIT; // Too short
            uint16_t received_crc = hmodbus->rx_buffer[hmodbus->rx_index - 2] | (hmodbus->rx_buffer[hmodbus->rx_index - 1] << 8);
            uint16_t calculated_crc = Modbus_CRC16(hmodbus->rx_buffer, hmodbus->rx_index - 2);
            
            if (received_crc != calculated_crc) {
                hmodbus->state = MB_STATE_IDLE;
                return MB_ERROR_CRC;
            }

            // Check Exception
            if (hmodbus->rx_buffer[1] & 0x80) {
                hmodbus->state = MB_STATE_IDLE;
                return MB_ERROR_EXCEPTION;
            }
            
            // Check Function Code Match
            if (hmodbus->rx_buffer[1] != hmodbus->pending_func_code) {
                 hmodbus->state = MB_STATE_IDLE;
                 return MB_ERROR_TRANSMIT;
            }

            // Success
            hmodbus->state = MB_STATE_IDLE;
            return MB_ERROR_NONE;
        }
    }
    return MB_ERROR_NONE;
}

#endif

/* ================================================================================== */
/*                                    SLAVE IMPLEMENTATION                            */
/* ================================================================================== */
#ifdef MODBUS_ENABLE_SLAVE

static void Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t func, uint8_t exc) {
    hmodbus->tx_buffer[0] = hmodbus->slave_id;
    hmodbus->tx_buffer[1] = func | 0x80;
    hmodbus->tx_buffer[2] = exc;
    
    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, 3);
    hmodbus->tx_buffer[3] = crc & 0xFF;
    hmodbus->tx_buffer[4] = (crc >> 8) & 0xFF;
    
    Modbus_Send(hmodbus, 5);
}

void Modbus_Slave_Listen(Modbus_Handle_t *hmodbus, uint16_t *register_map, uint16_t map_size) {
    if (hmodbus->frame_complete) {
        hmodbus->frame_complete = false;

        // 1. Minimum Length Check
        if (hmodbus->rx_index < 4) {
            hmodbus->rx_index = 0;
            // HAL_UART_Receive_IT called in IRQ
            return;
        }

        // 2. CRC Check
        uint16_t received_crc = hmodbus->rx_buffer[hmodbus->rx_index - 2] | (hmodbus->rx_buffer[hmodbus->rx_index - 1] << 8);
        uint16_t calculated_crc = Modbus_CRC16(hmodbus->rx_buffer, hmodbus->rx_index - 2);

        if (received_crc != calculated_crc) {
            // Silent drop on CRC error (standard Modbus behavior)
            hmodbus->rx_index = 0;
            return;
        }

        // 3. Slave ID Check
        if (hmodbus->rx_buffer[0] != hmodbus->slave_id) {
            hmodbus->rx_index = 0;
            return;
        }

        // 4. Parse PDU
        uint8_t func_code = hmodbus->rx_buffer[1];
        uint16_t start_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
        uint16_t count_val  = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5]; // Count for read, Value for write

        // 5. Handle Function Codes
        switch (func_code) {
            case MB_FC_READ_HOLDING_REGISTERS:
                if (start_addr + count_val > map_size) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDR);
                } else {
                    hmodbus->tx_buffer[0] = hmodbus->slave_id;
                    hmodbus->tx_buffer[1] = func_code;
                    hmodbus->tx_buffer[2] = count_val * 2; // Byte count

                    for (uint16_t i = 0; i < count_val; i++) {
                        hmodbus->tx_buffer[3 + i*2] = (register_map[start_addr + i] >> 8) & 0xFF;
                        hmodbus->tx_buffer[4 + i*2] = register_map[start_addr + i] & 0xFF;
                    }
                    
                    uint16_t len = 3 + count_val * 2;
                    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, len);
                    hmodbus->tx_buffer[len] = crc & 0xFF;
                    hmodbus->tx_buffer[len+1] = (crc >> 8) & 0xFF;
                    
                    Modbus_Send(hmodbus, len + 2);
                }
                break;

            case MB_FC_WRITE_SINGLE_REGISTER:
                if (start_addr >= map_size) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDR);
                } else {
                    // Update Register
                    register_map[start_addr] = count_val; // Value is in count_val position
                    
                    // Callback if needed
                    if (hmodbus->write_reg_cb) {
                        hmodbus->write_reg_cb(start_addr, count_val);
                    }

                    // Echo Response
                    Modbus_Send(hmodbus, 8); // Request and response are identical for FC06
                }
                break;

            default:
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
        }
        
        // Reset
        hmodbus->rx_index = 0;
        // Re-enable RX interrupt is handled in RxCplt logic or continuously running
    }
}

#endif

/* ================================================================================== */
/*                             INTERRUPT HANDLERS                                     */
/* ================================================================================== */

void Modbus_IRQHandler_RxCplt(Modbus_Handle_t *hmodbus) {
    // 1. Reset T3.5 Timer
    __HAL_TIM_SET_COUNTER(hmodbus->htim, 0);
    HAL_TIM_Base_Start_IT(hmodbus->htim);

    // 2. Store Byte
    hmodbus->rx_index++;
    if (hmodbus->rx_index >= MODBUS_MAX_ADU_SIZE) {
        hmodbus->rx_index = 0; // Buffer overflow protection
    }

    // 3. Listen for next byte
    HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[hmodbus->rx_index], 1);
}

void Modbus_IRQHandler_Timeout(Modbus_Handle_t *hmodbus) {
    // T3.5 Expired -> Frame Complete
    HAL_TIM_Base_Stop_IT(hmodbus->htim);
    hmodbus->frame_complete = true;
    hmodbus->state = MB_STATE_PROCESSING;
}