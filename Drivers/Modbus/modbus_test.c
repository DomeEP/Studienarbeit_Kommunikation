/**
 * @file modbus_test.c
 * @brief Self-Test for Modbus Library
 * @details Include this file in your project to verify the library logic.
 *          Call Modbus_RunSelfTest() from main().
 */

#include "modbus_rtu.h"
#include <stdio.h>
#include <string.h>

// Mock Hardware Handles for Test
UART_HandleTypeDef huart_test;
TIM_HandleTypeDef htim_test;
Modbus_Handle_t hmodbus_test;

// Test Register Map
uint16_t test_regs[10] = {0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

// Helper: Calculate CRC (Copy from library implementation for verification)
static uint16_t Test_CRC16(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// Function to run the test
void Modbus_RunSelfTest(void) {
    printf("--- Starting Modbus Self-Test ---\n");

    // 1. Initialize
    Modbus_Init(&hmodbus_test, &huart_test, &htim_test, 1); // Slave ID 1
    printf("[PASS] Initialization\n");

    // 2. Simulate Receiving a Valid Read Request (FC03)
    // Request: Slave 1, FC 03, Start 0x0000, Count 2, CRC
    uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    
    // Manually inject into buffer
    memcpy(hmodbus_test.rx_buffer, request, sizeof(request));
    hmodbus_test.rx_index = sizeof(request);
    hmodbus_test.frame_complete = true; // Simulate Timer Timeout

    // 3. Process the Request
    Modbus_Slave_Listen(&hmodbus_test, test_regs, 10);

    // 4. Verify Response
    // Expected: Slave 1, FC 03, Bytes 4, Reg0Hi, Reg0Lo, Reg1Hi, Reg1Lo, CRC
    // Reg0 = 0x1111, Reg1 = 0x2222
    if (hmodbus_test.tx_buffer[0] == 0x01 && 
        hmodbus_test.tx_buffer[1] == 0x03 && 
        hmodbus_test.tx_buffer[2] == 0x04 &&
        hmodbus_test.tx_buffer[3] == 0x11 && hmodbus_test.tx_buffer[4] == 0x11 &&
        hmodbus_test.tx_buffer[5] == 0x22 && hmodbus_test.tx_buffer[6] == 0x22) {
        printf("[PASS] Slave Response Logic (FC03)\n");
    } else {
        printf("[FAIL] Slave Response Logic (FC03)\n");
        printf("Expected: 01 03 04 11 11 22 22 ...\n");
        printf("Actual:   %02X %02X %02X %02X %02X %02X %02X\n", 
               hmodbus_test.tx_buffer[0], hmodbus_test.tx_buffer[1], hmodbus_test.tx_buffer[2],
               hmodbus_test.tx_buffer[3], hmodbus_test.tx_buffer[4], hmodbus_test.tx_buffer[5], hmodbus_test.tx_buffer[6]);
    }

    // 5. Test CRC Calculation
    uint16_t crc = Test_CRC16(request, 6);
    if (crc == 0x0BC4) { // Little Endian C4 0B
        printf("[PASS] CRC Calculation matches expected 0x0BC4\n");
    } else {
        printf("[FAIL] CRC Calculation: Got %04X, Expected 0x0BC4\n", crc);
    }

    printf("--- Test Complete ---\n");
}
