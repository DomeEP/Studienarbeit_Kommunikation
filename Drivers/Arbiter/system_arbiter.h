/**
 * @file system_arbiter.h
 * @brief Central Modbus master (arbiter) for the T3200 microgrid
 *
 * Auto-discovers slaves, polls cyclically, computes a global operating
 * mode via voting, and distributes the result back to all nodes.
 */

#ifndef SYSTEM_ARBITER_H
#define SYSTEM_ARBITER_H

#include "stm32g4xx_hal.h"
#include "slave_node.h"
#include <stdint.h>
#include <stdbool.h>

#define ARBITER_MAX_SLAVES 10

void     Arbiter_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void     Arbiter_Process(void);
void     Arbiter_SetGlobalRunState(bool run);
void     Arbiter_TriggerWakeup(void);
bool     Arbiter_GetGlobalRunState(void);
uint16_t Arbiter_GetSystemMode(void);

#endif /* SYSTEM_ARBITER_H */
