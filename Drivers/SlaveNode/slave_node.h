/**
 * @file slave_node.h
 * @brief Plug-and-Play Treiber für Slave-Knoten (Inverter / DC-DC)
 * 
 * === INTEGRATIONS-ANLEITUNG FÜR ANDERE GRUPPEN ===
 * 
 * 1. DATEIEN KOPIEREN:
 *    Kopiere die Ordner `Drivers/Modbus` und `Drivers/SlaveNode` in dein Projekt.
 * 
 * 2. CUBE-MX EINSTELLUNGEN:
 *    - RS485-Schnittstelle (Connectivity -> USARTx):
 *       * Mode: "Asynchronous" wählen.
 *       * RS485 Hardware Flow Control: "Enable" (wichtig für die DE-Pin Steuerung).
 *       * Baudrate: 115200 Bits/s.
 *       * NVIC Settings: "USARTx global interrupt" -> Enabled.
 *    - Timer (Internal Timers -> TIMx, z.B. TIM6):
 *       * Activated: Checkbox "Activated" (bzw. Internal Clock) wählen.
 *       * Prescaler & Period: Beliebige Werte (der Treiber überschreibt diese automatisch).
 *       * NVIC Settings: "TIMx global interrupt" -> Enabled.
 *    - Interrupt-Prioritäten (System Core -> NVIC):
 *       * Der TIMER-Interrupt MUSS eine höhere Priorität haben als der UART-Interrupt!
 *       * Beispiel: TIMx = Prio 0, USARTx = Prio 1.
 * 
 * 3. INTERRUPTS VERBINDEN (in `stm32g4xx_it.c`):
 *    Importiere das Modbus-Handle: `extern Modbus_Handle_t hmodbus;`
 *    - In `USARTx_IRQHandler`: `Modbus_IRQHandler_RxCplt(&hmodbus);`
 *    - In `TIMx_IRQHandler`:   `Modbus_IRQHandler_Timeout(&hmodbus);`
 *    - In `USARTx_IRQHandler` (bei Error): `Modbus_IRQHandler_Error(&hmodbus);`
 * 
 * 4. INITIALISIERUNG (in `main.c`):
 *    - `SlaveNode_Init(&huart2, &htim6, APP_SLAVE_ID, MyEmergencyCallback);`
 *    - In der `while(1)`-Hauptschleife: `SlaveNode_Process();`
 */

#ifndef SLAVE_NODE_H
#define SLAVE_NODE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Definiert die drei möglichen Systemzustände (Wunsch / Erlaubnis)
 */
#define SYSTEM_MODE_STANDBY   0
#define SYSTEM_MODE_CHARGE    1
#define SYSTEM_MODE_DISCHARGE 2
#define SYSTEM_MODE_WAKEUP    3

#define NODE_TYPE_UNKNOWN     0
#define NODE_TYPE_INVERTER    1
#define NODE_TYPE_DCDC        2

/**
 * @brief Signatur für den Notaus-Callback
 * Wird aufgerufen, wenn der Master einen sofortigen Stop befiehlt.
 */
typedef void (*SlaveNode_EmergencyCallback_t)(void);

/**
 * @brief Initialisiert den Slave Node und startet die Kommunikation.
 * 
 * @param huart Pointer zum verwendeten RS485 UART-Handle (z.B. &huart2)
 * @param htim  Pointer zum verwendeten Timer-Handle (z.B. &htim6)
 * @param slave_id Eigene Modbus-Adresse (1=Inverter, 2=DC/DC #1, 3=DC/DC #2, ...)
 * @param on_emergency Funktion, die bei System-Notaus aufgerufen werden soll (oder NULL)
 */
void SlaveNode_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id, SlaveNode_EmergencyCallback_t on_emergency);

/**
 * @brief Hält die Modbus State-Machine am Leben.
 * MUSS zyklisch in der Haupt-while(1)-Schleife aufgerufen werden.
 */
void SlaveNode_Process(void);

/**
 * @brief Teilt dem Master den eigenen, durch physikalische Sensoren ermittelten Wunsch mit.
 * 
 * @param mode SYSTEM_MODE_STANDBY, SYSTEM_MODE_CHARGE oder SYSTEM_MODE_DISCHARGE
 */
void SlaveNode_SetDesiredMode(uint16_t mode);

/**
 * @brief Deklariert dem Master, welcher Hardware-Knotentyp man ist.
 * 
 * @param type NODE_TYPE_INVERTER oder NODE_TYPE_DCDC
 */
void SlaveNode_SetNodeType(uint16_t type);

/**
 * @brief Meldet einen lokalen Systemfehler an den Master (z.B. Überstrom).
 * Führt dazu, dass der Master sofort den Notaus für das gesamte System erzwingt.
 * 
 * @param is_emergency true = Fehler aktiv, false = Alles in Ordnung
 */
void SlaveNode_SetEmergencyFlag(bool is_emergency);

/**
 * @brief Lies den aktuell vom Master genehmigten und kommandierten Betriebsmodus aus.
 * Die Hardware der Gruppe DARF NUR auf Basis DIESES Werts schalten, niemals direkt auf Basis des eigenen Wunsches!
 * 
 * @return SYSTEM_MODE_STANDBY, SYSTEM_MODE_CHARGE oder SYSTEM_MODE_DISCHARGE
 */
uint16_t SlaveNode_GetAllowedMode(void);

/**
 * @brief Wie GetAllowedMode, aber setzt spezielle Trigger-Modi (wie WAKEUP) nach dem Auslesen sofort zurück,
 * um Mehrfachauslösungen in schnellen Schleifen zu verhindern.
 * 
 * @return Den gelesenen Modus.
 */
uint16_t SlaveNode_ConsumeAllowedMode(void);

#endif /* SLAVE_NODE_H */
