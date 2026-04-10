/**
 * @file system_arbiter.h
 * @brief Zentraler Modbus-Master Treiber (Arbiter)
 * 
 * Verarbeitet das zyklische Polling einer Liste von Slaves,
 * sammelt deren Zustands-Wünsche und berechnet eine Freigabe
 * (Voting-Mechanismus). Sendet die Freigabe an alle Slaves zurück.
 */

#ifndef SYSTEM_ARBITER_H
#define SYSTEM_ARBITER_H

#include "stm32g4xx_hal.h"
#include "slave_node.h" // Für die T3100_MODE Defines
#include <stdint.h>
#include <stdbool.h>

#define ARBITER_MAX_SLAVES 10

/**
 * @brief Initialisiert den Arbiter (Modbus Master) mit Auto-Discovery.
 * 
 * @param huart Pointer zum RS485 UART Handle
 * @param htim  Pointer zum Timer Handle
 */
void Arbiter_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

/**
 * @brief State-Machine des Arbiters. 
 * MUSS endlos in der Hauptschleife aufgerufen werden.
 */
void Arbiter_Process(void);

/**
 * @brief Globale Freigabe für das System. 
 * True = System darf gemäß Slaves laufen. False = System wird hart in den Standby gezwungen (Notaus).
 */
void Arbiter_SetGlobalRunState(bool run);

/**
 * @brief Löst für einen einzigen Sendezyklus an alle Slaves den Wakeup-Modus aus.
 */
void Arbiter_TriggerWakeup(void);

/**
 * @brief Gibt zurück, ob das System generell eingeschaltet ist.
 */
bool Arbiter_GetGlobalRunState(void);

/**
 * @brief Gibt den aktuell vom Arbiter für das Gesamtsystem errechneten Modus zurück.
 * @return SYSTEM_MODE_STANDBY, SYSTEM_MODE_CHARGE oder SYSTEM_MODE_DISCHARGE
 */
uint16_t Arbiter_GetSystemMode(void);

#endif /* SYSTEM_ARBITER_H */
