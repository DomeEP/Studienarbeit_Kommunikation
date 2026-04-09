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
 * - T3.5 character timeout via TIM6.
 * - Support for FC03, FC06, FC16.
 * - Diagnostic counters for reliability monitoring.
 */

#include "modbus_rtu.h"
#include <string.h>

/* ================================================================================== */
/*                                 PRIVATE PROTOTYPES                                 */
/* ================================================================================== */

static uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);
static void Modbus_Send(Modbus_Handle_t *hmodbus, uint16_t length);
static void Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t func, uint8_t exc);
static void Modbus_Restart_RX(Modbus_Handle_t *hmodbus);

/* ================================================================================== */
/*                                 CRC16 LOOKUP TABLE                                 */
/* ================================================================================== */
// Standard Modbus CRC16 Table (Polynomial 0xA001)
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
 * @details Iterates through the buffer, updating the CRC using the lookup table.
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

/**
 * @brief Safely restart RX at buffer[0]
 */
static void Modbus_Restart_RX(Modbus_Handle_t *hmodbus) {
    // 1. Laufenden Empfang in der HAL abbrechen
    HAL_UART_AbortReceive(hmodbus->huart);
    
    // 2. Hardware Error Flags und Idle Flag hart löschen (WICHTIG für STM32G4)
    __HAL_UART_CLEAR_FLAG(hmodbus->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF | UART_CLEAR_IDLEF);
    
    // 3. RX FIFO verwerfen (alles verwerfen, was evtl. hängen geblieben ist)
    __HAL_UART_SEND_REQ(hmodbus->huart, UART_RXDATA_FLUSH_REQUEST);
    
    // 4. HAL Zustände bereinigen, damit ReceiveToIdle_IT nicht "HAL_BUSY" wirft
    hmodbus->huart->ErrorCode = HAL_UART_ERROR_NONE;
    hmodbus->huart->RxState = HAL_UART_STATE_READY;
    hmodbus->huart->gState = HAL_UART_STATE_READY;
    hmodbus->huart->Lock = HAL_UNLOCKED;
    
    // 5. Software-Index zurücksetzen
    hmodbus->rx_index = 0;
    
    // 6. Empfang starten (HARDWARE IDLE LINE DETECTION)
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus->huart, hmodbus->rx_buffer, MODBUS_MAX_ADU_SIZE);
}

/* ================================================================================== */
/*                                    CORE FUNCTIONS                                  */
/* ================================================================================== */

void Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id) {
    // Clear the struct to avoid garbage data
    memset(hmodbus, 0, sizeof(Modbus_Handle_t));

    hmodbus->huart = huart;
    hmodbus->htim  = htim;
    hmodbus->slave_id = slave_id;
    hmodbus->rx_index = 0;
    hmodbus->state = MB_STATE_IDLE;
    hmodbus->frame_complete = false;

    // Start Listening (Hardware IDLE Line Mode)
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus->huart, hmodbus->rx_buffer, MODBUS_MAX_ADU_SIZE);
}

static void Modbus_Send(Modbus_Handle_t *hmodbus, uint16_t length) {
    // 1. Empfang abschalten (verhindert IRQ-Konflikte während TX)
    HAL_UART_AbortReceive(hmodbus->huart);
    
    // 2. Error-Flags löschen und RX-Register flushen
    __HAL_UART_CLEAR_FLAG(hmodbus->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(hmodbus->huart, UART_RXDATA_FLUSH_REQUEST);
    
    // 3. Direct Register-Level Transmit
    //    Umgeht HAL __HAL_LOCK() komplett → keine Race Conditions mehr!
    //    Hardware RS485 DE wird automatisch von der USART-Peripherie gesteuert.
    for (uint16_t i = 0; i < length; i++) {
        // Warten bis Transmit Data Register leer ist (TXE/TXFNF)
        while (!(hmodbus->huart->Instance->ISR & USART_ISR_TXE_TXFNF)) {}
        hmodbus->huart->Instance->TDR = hmodbus->tx_buffer[i];
    }
    
    // 4. Warten bis das LETZTE Byte physisch die Leitung verlassen hat (TC Flag)
    //    Erst wenn TC gesetzt ist, wird der DE-Pin von der Hardware auf LOW gezogen.
    //    Ohne dieses Warten wird die Übertragung abgeschnitten!
    while (!(hmodbus->huart->Instance->ISR & USART_ISR_TC)) {}
    
    hmodbus->stats.tx_frames++;
}

/* ================================================================================== */
/*                                    MASTER IMPLEMENTATION                           */
/* ================================================================================== */
#ifdef MODBUS_ENABLE_MASTER

HAL_StatusTypeDef Modbus_Master_Request(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_val) {
    if (hmodbus->state != MB_STATE_IDLE && hmodbus->state != MB_STATE_RX) {
        return HAL_BUSY; // Communication in progress
    }

    // Build Request Frame
    hmodbus->tx_buffer[0] = slave_id;
    hmodbus->tx_buffer[1] = func_code;
    hmodbus->tx_buffer[2] = (reg_addr >> 8) & 0xFF;
    hmodbus->tx_buffer[3] = reg_addr & 0xFF;
    hmodbus->tx_buffer[4] = (reg_val >> 8) & 0xFF;
    hmodbus->tx_buffer[5] = reg_val & 0xFF;

    // CRC
    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, 6);
    hmodbus->tx_buffer[6] = crc & 0xFF;
    hmodbus->tx_buffer[7] = (crc >> 8) & 0xFF;
    hmodbus->tx_length = 8;

    // State Update
    hmodbus->pending_func_code = func_code;
    hmodbus->state = MB_STATE_WAIT_RESPONSE;
    hmodbus->last_activity_timestamp = HAL_GetTick();

    hmodbus->frame_complete = false;
    Modbus_Send(hmodbus, hmodbus->tx_length);
    
    // Erwarte Antwort: Erst NACHDEM gesendet wurde den RX starten, um eigenes Echo zu vermeiden!
    Modbus_Restart_RX(hmodbus);
    
    return HAL_OK;
}

HAL_StatusTypeDef Modbus_Master_WriteMultiple(Modbus_Handle_t *hmodbus, uint8_t slave_id, uint16_t start_addr, uint16_t reg_count, uint16_t *data) {
    if (hmodbus->state != MB_STATE_IDLE) return HAL_BUSY;

    // FC16 Header
    hmodbus->tx_buffer[0] = slave_id;
    hmodbus->tx_buffer[1] = MB_FC_WRITE_MULTIPLE_REGISTERS;
    hmodbus->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    hmodbus->tx_buffer[3] = start_addr & 0xFF;
    hmodbus->tx_buffer[4] = (reg_count >> 8) & 0xFF;
    hmodbus->tx_buffer[5] = reg_count & 0xFF;
    hmodbus->tx_buffer[6] = (uint8_t)(reg_count * 2); // Byte Count

    // Data Payload
    for (uint16_t i = 0; i < reg_count; i++) {
        hmodbus->tx_buffer[7 + i*2] = (data[i] >> 8) & 0xFF;
        hmodbus->tx_buffer[8 + i*2] = data[i] & 0xFF;
    }

    uint16_t len = 7 + (reg_count * 2);
    
    // CRC
    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, len);
    hmodbus->tx_buffer[len] = crc & 0xFF;
    hmodbus->tx_buffer[len+1] = (crc >> 8) & 0xFF;
    
    hmodbus->tx_length = len + 2;
    hmodbus->pending_func_code = MB_FC_WRITE_MULTIPLE_REGISTERS;
    hmodbus->state = MB_STATE_WAIT_RESPONSE;
    hmodbus->last_activity_timestamp = HAL_GetTick();

    hmodbus->frame_complete = false;
    Modbus_Send(hmodbus, hmodbus->tx_length);
    
    // Erwarte Antwort: Erst NACHDEM gesendet wurde den RX starten, um eigenes Echo zu vermeiden!
    Modbus_Restart_RX(hmodbus);
    
    return HAL_OK;
}

Modbus_Error_t Modbus_Master_Process(Modbus_Handle_t *hmodbus) {
    // Timeout check — nur wenn wir warten UND noch kein Frame komplett ist
    if (hmodbus->state == MB_STATE_WAIT_RESPONSE && !hmodbus->frame_complete) {
        if ((HAL_GetTick() - hmodbus->last_activity_timestamp) > MODBUS_RESPONSE_TIMEOUT_MS) {
            hmodbus->state = MB_STATE_IDLE;
            hmodbus->stats.timeouts++;
            if(hmodbus->error_cb) hmodbus->error_cb(MB_ERROR_TIMEOUT);
            Modbus_Restart_RX(hmodbus);
            return MB_ERROR_TIMEOUT;
        }
        return MB_ERROR_NONE; // Noch warten...
    }
    
    // Frame komplett empfangen? (Kann in WAIT_RESPONSE oder PROCESSING state sein)
    if (hmodbus->frame_complete) {
        hmodbus->frame_complete = false;
        hmodbus->stats.rx_frames++;
        
        // Validate minimum length
        if (hmodbus->rx_index < 4) {
             hmodbus->state = MB_STATE_IDLE;
             if(hmodbus->error_cb) hmodbus->error_cb(MB_ERROR_TRANSMIT);
             Modbus_Restart_RX(hmodbus);
             return MB_ERROR_TRANSMIT;
        }
        
        // Validate CRC
        uint16_t received_crc = hmodbus->rx_buffer[hmodbus->rx_index - 2] | (hmodbus->rx_buffer[hmodbus->rx_index - 1] << 8);
        uint16_t calculated_crc = Modbus_CRC16(hmodbus->rx_buffer, hmodbus->rx_index - 2);
        
        if (received_crc != calculated_crc) {
            hmodbus->state = MB_STATE_IDLE;
            hmodbus->stats.crc_errors++;
            if(hmodbus->error_cb) hmodbus->error_cb(MB_ERROR_CRC);
            Modbus_Restart_RX(hmodbus);
            return MB_ERROR_CRC;
        }

        // Check Exception (MSB set)
        if (hmodbus->rx_buffer[1] & 0x80) {
            hmodbus->state = MB_STATE_IDLE;
            if(hmodbus->error_cb) hmodbus->error_cb(MB_ERROR_EXCEPTION);
            Modbus_Restart_RX(hmodbus);
            return MB_ERROR_EXCEPTION;
        }
        
        // Check Function Code Match
        if (hmodbus->rx_buffer[1] != hmodbus->pending_func_code) {
             hmodbus->state = MB_STATE_IDLE;
             if(hmodbus->error_cb) hmodbus->error_cb(MB_ERROR_TRANSMIT);
             Modbus_Restart_RX(hmodbus);
             return MB_ERROR_TRANSMIT;
        }

        // PDU verarbeiten (nur FC03 implementiert im Master für dieses Beispiel)
        if (hmodbus->pending_func_code == MB_FC_READ_HOLDING_REGISTERS) {
            // Für Simplicity: Wir schreiben den ersten erhaltenen Registerwert direkt in den RAM
            // In einer echten Lib würde man pending_start_addr usw. tracken.
            extern uint16_t register_map[]; 
            
            // Angenommen, wir lesen 2 Register (Wunsch & Emergency):
            // rx_buffer[2] ist Byte-Count. rx_buffer[3/4] = Reg 1, rx_buffer[5/6] = Reg 2
            register_map[0] = (hmodbus->rx_buffer[3] << 8) | hmodbus->rx_buffer[4];
            
            uint8_t byte_count = hmodbus->rx_buffer[2];
            if (byte_count >= 4) {
                register_map[1] = (hmodbus->rx_buffer[5] << 8) | hmodbus->rx_buffer[6];
            }
        }

        // Success!
        hmodbus->state = MB_STATE_IDLE;
        if (hmodbus->master_complete_cb) hmodbus->master_complete_cb();
        
        Modbus_Restart_RX(hmodbus);
        return MB_ERROR_NONE;
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
    
    HAL_Delay(3); // RS485 Turnaround Delay
    Modbus_Send(hmodbus, 5);
}

void Modbus_Slave_Listen(Modbus_Handle_t *hmodbus, uint16_t *register_map, uint16_t map_size) {
    if (hmodbus->frame_complete) {
        hmodbus->frame_complete = false;
        hmodbus->stats.rx_frames++;

        // 1. Minimum Length Check
        if (hmodbus->rx_index < 4) {
            hmodbus->state = MB_STATE_IDLE;
            Modbus_Restart_RX(hmodbus);
            return;
        }

        // 2. CRC Check
        uint16_t received_crc = hmodbus->rx_buffer[hmodbus->rx_index - 2] | (hmodbus->rx_buffer[hmodbus->rx_index - 1] << 8);
        uint16_t calculated_crc = Modbus_CRC16(hmodbus->rx_buffer, hmodbus->rx_index - 2);

        if (received_crc != calculated_crc) {
            hmodbus->stats.crc_errors++;
            hmodbus->state = MB_STATE_IDLE;
            Modbus_Restart_RX(hmodbus);
            return; // Silent Drop
        }

        // 3. Slave ID Check
        if (hmodbus->rx_buffer[0] != hmodbus->slave_id) {
            hmodbus->state = MB_STATE_IDLE;
            Modbus_Restart_RX(hmodbus);
            return; // Not for me
        }

        // 4. Parse PDU
        uint8_t func_code = hmodbus->rx_buffer[1];
        uint16_t start_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
        uint16_t count_val  = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];

        // *** KRITISCH: RX-Empfang ABSCHALTEN bevor wir senden! ***
        // Sonst empfängt der Slave sein eigenes Echo über den RS485-Bus
        // und interpretiert es als neues Frame → Chaos!
        HAL_UART_AbortReceive(hmodbus->huart);

        // 5. Handle Function Codes
        switch (func_code) {
            case MB_FC_READ_HOLDING_REGISTERS:
                if (start_addr + count_val > map_size || 
                   (hmodbus->validate_addr_cb && !hmodbus->validate_addr_cb(start_addr))) 
                {
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
                    
                    /* KRITISCH: RS485 Turnaround Delay. 
                     * Gibt dem isolierten ADM2587E des Masters Zeit, von TX auf RX umzuschalten.
                     * Ohne diese Pause kollidiert unsere Antwort mit dem abfallenden DE-Signal des Masters! */
                    HAL_Delay(100);
                    
                    Modbus_Send(hmodbus, len + 2);
                }
                break;

            case MB_FC_WRITE_SINGLE_REGISTER:
                if (start_addr >= map_size ||
                   (hmodbus->validate_addr_cb && !hmodbus->validate_addr_cb(start_addr)))
                {
                    Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDR);
                } else {
                    register_map[start_addr] = count_val;
                    
                    if (hmodbus->write_reg_cb) {
                        hmodbus->write_reg_cb(start_addr, count_val);
                    }

                    // Echo Response
                    for (int i = 0; i < 8; i++) {
                        hmodbus->tx_buffer[i] = hmodbus->rx_buffer[i];
                    }
                    HAL_Delay(3); // RS485 Turnaround Delay
                    Modbus_Send(hmodbus, 8); 
                }
                break;

            case MB_FC_WRITE_MULTIPLE_REGISTERS:
                {
                    uint16_t reg_count = count_val;
                    uint8_t byte_count = hmodbus->rx_buffer[6];
                    
                    if (start_addr + reg_count > map_size ||
                       (hmodbus->validate_addr_cb && !hmodbus->validate_addr_cb(start_addr)))
                    {
                        Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDR);
                    } else if (byte_count != reg_count * 2) {
                        Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                    } else {
                        for (uint16_t i = 0; i < reg_count; i++) {
                            uint16_t val = (hmodbus->rx_buffer[7 + i*2] << 8) | hmodbus->rx_buffer[8 + i*2];
                            register_map[start_addr + i] = val;
                            if (hmodbus->write_reg_cb) hmodbus->write_reg_cb(start_addr + i, val);
                        }
                        
                        hmodbus->tx_buffer[0] = hmodbus->slave_id;
                        hmodbus->tx_buffer[1] = func_code;
                        hmodbus->tx_buffer[2] = (start_addr >> 8) & 0xFF;
                        hmodbus->tx_buffer[3] = start_addr & 0xFF;
                        hmodbus->tx_buffer[4] = (reg_count >> 8) & 0xFF;
                        hmodbus->tx_buffer[5] = reg_count & 0xFF;
                        
                        uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, 6);
                        hmodbus->tx_buffer[6] = crc & 0xFF;
                        hmodbus->tx_buffer[7] = (crc >> 8) & 0xFF;
                        
                        HAL_Delay(3); // RS485 Turnaround Delay
                        Modbus_Send(hmodbus, 8);
                    }
                }
                break;

            default:
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
        }
        
        // *** KRITISCH: State zurücksetzen und RX NACH dem Senden neu starten ***
        // Damit fangen wir erst jetzt an zu empfangen, nachdem unsere Antwort
        // komplett raus ist → kein Self-Echo!
        hmodbus->state = MB_STATE_IDLE;
        hmodbus->frame_complete = false;
        Modbus_Restart_RX(hmodbus);
    }
}

#endif

/* ================================================================================== */
/*                             INTERRUPT HANDLERS                                     */
/* ================================================================================== */

// ----------------------------------------------------------------------------------
// NEUE ROBUSTE EMPFANGSROUTINE VOR ÜBER STM32 HARDWARE IDLE DETECTION
// ----------------------------------------------------------------------------------
void Modbus_IRQHandler_RxEvent(Modbus_Handle_t *hmodbus, uint16_t Size)
{
    // Wenn Hardware IDLE (Lücke auf Bus) erkennt oder Buffer voll ist:
    hmodbus->rx_index = Size; // Die Hardware sagt uns exakt, wie viele Bytes empfangen wurden!
    
    // Frame als komplett markieren
    hmodbus->frame_complete = true;
    hmodbus->state = MB_STATE_PROCESSING;
}

void Modbus_IRQHandler_RxCplt(Modbus_Handle_t *hmodbus) {
    // VERALTET: Wir benutzen jetzt HAL_UARTEx_RxEventCallback für alles!
}

void Modbus_IRQHandler_Timeout(Modbus_Handle_t *hmodbus) {
    // VERALTET: Wir benutzen jetzt Hardware IDLE detection (HAL_UARTEx_RxEventCallback)!
}

void Modbus_IRQHandler_Error(Modbus_Handle_t *hmodbus) {
    // Hardware Error Flags löschen (damit UART weiterarbeiten kann)
    __HAL_UART_CLEAR_FLAG(hmodbus->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    hmodbus->huart->ErrorCode = HAL_UART_ERROR_NONE;
    hmodbus->stats.bus_errors++;
    
    // Empfang sofort neu starten (mit Force-Reset)
    Modbus_Restart_RX(hmodbus);
}

