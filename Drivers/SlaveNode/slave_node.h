/**
 * @file slave_node.h
 * @brief High-level API for slave nodes in the T3200 microgrid
 *
 * Wraps the Modbus register map into a simple interface for subsystem teams.
 * See Integration_Guide_Slave.md for setup instructions.
 */

#ifndef SLAVE_NODE_H
#define SLAVE_NODE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_MODE_STANDBY   0
#define SYSTEM_MODE_CHARGE    1
#define SYSTEM_MODE_DISCHARGE 2
#define SYSTEM_MODE_WAKEUP    3

#define NODE_TYPE_UNKNOWN     0
#define NODE_TYPE_INVERTER    1
#define NODE_TYPE_DCDC        2

typedef void (*SlaveNode_EmergencyCallback_t)(void);

void     SlaveNode_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id, SlaveNode_EmergencyCallback_t on_emergency);
void     SlaveNode_Process(void);
void     SlaveNode_SetDesiredMode(uint16_t mode);
void     SlaveNode_SetNodeType(uint16_t type);
void     SlaveNode_SetEmergencyFlag(bool is_emergency);
uint16_t SlaveNode_GetAllowedMode(void);
uint16_t SlaveNode_ConsumeAllowedMode(void);

#endif /* SLAVE_NODE_H */
