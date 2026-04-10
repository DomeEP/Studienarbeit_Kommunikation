/**
 * @file slave_node.c
 * @brief Implementierung des Slave Node Treibers
 */

#include "slave_node.h"
#include "modbus_rtu.h"
#include "app_config.h"

// Das globale Modbus Handle für diesen Node (wird im Interrupt-Handler benötigt)
Modbus_Handle_t hmodbus; 

// Die physikalische Register Map, die über Modbus zwischen Master/Slave geteilt wird
uint16_t register_map[APP_REGISTER_MAP_SIZE] = {0};

// Interner Pointer auf den Emergency Callback
static SlaveNode_EmergencyCallback_t user_emergency_cb = NULL;

/* 
 * Interner Callback, der von der modbus_rtu Lib aufgerufen wird, 
 * wenn der Master erfolgreich Register (FC06/FC16) geschrieben hat.
 */
static void internal_on_register_write(uint16_t reg_addr, uint16_t value) {
    if (reg_addr == REG_SAFETY_STOP && value == 1) {
        // Notaus befohlen! Alles ins Standby zwingen.
        register_map[REG_ERLAUBNIS] = SYSTEM_MODE_STANDBY;
        
        // Applikation informieren, falls Callback registriert
        if (user_emergency_cb != NULL) {
            user_emergency_cb();
        }
    }
}

void SlaveNode_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_id, SlaveNode_EmergencyCallback_t on_emergency) {
    user_emergency_cb = on_emergency;

    // Register auf sichere Default-Werte setzen
    register_map[REG_WUNSCH]    = SYSTEM_MODE_STANDBY;
    register_map[REG_EMERGENCY] = 0; // Kein Fehler
    register_map[REG_NODE_TYPE] = NODE_TYPE_UNKNOWN;
    register_map[REG_ERLAUBNIS] = SYSTEM_MODE_STANDBY;

    // Modbus Stack initialisieren
    Modbus_Init(&hmodbus, huart, htim, slave_id);

    // Write Callback registrieren
    hmodbus.write_reg_cb = internal_on_register_write;
}

void SlaveNode_Process(void) {
    // Treibt die Modbus-Library an (Horcht auf Requests und beantwortet sie)
    Modbus_Slave_Listen(&hmodbus, register_map, APP_REGISTER_MAP_SIZE);
}

void SlaveNode_SetNodeType(uint16_t type) {
    register_map[REG_NODE_TYPE] = type;
}

void SlaveNode_SetDesiredMode(uint16_t mode) {
    if (mode <= SYSTEM_MODE_DISCHARGE) {
        register_map[REG_WUNSCH] = mode;
    }
}

void SlaveNode_SetEmergencyFlag(bool is_emergency) {
    register_map[REG_EMERGENCY] = is_emergency ? 1 : 0;
}

uint16_t SlaveNode_GetAllowedMode(void) {
    return register_map[REG_ERLAUBNIS];
}

uint16_t SlaveNode_ConsumeAllowedMode(void) {
    uint16_t current = register_map[REG_ERLAUBNIS];
    if (current == SYSTEM_MODE_WAKEUP) {
        // Konsumieren: Verhindert Dauer-Blink-Schleife
        register_map[REG_ERLAUBNIS] = SYSTEM_MODE_STANDBY;
    }
    return current;
}
