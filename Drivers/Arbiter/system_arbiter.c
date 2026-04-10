/**
 * @file system_arbiter.c
 * @brief Implementierung des System Arbiters mit Auto-Discovery,
 *        Inverter-Priorität und ITM/SWO Live-Dashboard.
 */

#include "system_arbiter.h"
#include "modbus_rtu.h"
#include "app_config.h"
#include <string.h>
#include <stdio.h>

/* ================================================================================== */
/*                            ITM / SWO DEBUG OUTPUT                                  */
/* ================================================================================== */

/**
 * ITM_SendChar schreibt ein einzelnes Zeichen auf den SWO-Debug-Pin.
 * Das funktioniert über das gleiche USB-Kabel, mit dem auch geflasht wird.
 * In der STM32CubeIDE kann man den Output unter:
 *   Window -> Show View -> SWV -> SWV ITM Data Console
 * ansehen (Port 0 aktivieren, SWV Clock = 16 MHz bei HSI).
 */
static void ITM_Print(const char *str) {
    while (*str) {
        ITM_SendChar(*str++);
    }
}

static char itm_buf[128]; // Interner Puffer für formatierten Output

/* ================================================================================== */
/*                               MODBUS HANDLE & MAP                                  */
/* ================================================================================== */

// Das globale Modbus-Master-Handle
Modbus_Handle_t hmodbus;

// In diese Map schreibt modbus_rtu.c per `extern` nach einem erfolgreichen FC03
uint16_t register_map[APP_REGISTER_MAP_SIZE] = {0};

/* ================================================================================== */
/*                            INTERNE DATENSTRUKTUREN                                 */
/* ================================================================================== */

typedef struct {
    uint16_t wunsch;
    uint16_t emergency;
    uint16_t node_type;   // NODE_TYPE_INVERTER oder NODE_TYPE_DCDC
} SlaveState_t;

static SlaveState_t slave_states[ARBITER_MAX_SLAVES];
static uint8_t  active_slaves[ARBITER_MAX_SLAVES];
static uint8_t  slave_count = 0;

/* ================================================================================== */
/*                              STATE MACHINE                                         */
/* ================================================================================== */

typedef enum {
    AS_DISCOVER = 0,      // Boot: Ping IDs 1..10
    AS_DISCOVER_WAIT,     // Warte auf Antwort oder Timeout
    AS_POLL_RX,           // Normalbetrieb: Lese Slave-Daten
    AS_WAIT_RX,
    AS_EVALUATE,
    AS_POLL_TX,
    AS_WAIT_TX,
    AS_DELAY,
    AS_ERROR
} ArbiterState_t;

static ArbiterState_t current_state = AS_DISCOVER;
static uint8_t target_idx = 0;
static uint32_t delay_tick = 0;

// Discovery: Welche ID wird gerade geprüft?
static uint8_t discovery_current_id = 1;
#define DISCOVERY_MAX_ID 10
// Kurzer Timeout nur für Discovery (normaler ist 1000ms, das wäre zu langsam)
#define DISCOVERY_TIMEOUT_MS 150

static uint16_t global_allowed_mode = SYSTEM_MODE_STANDBY;
static bool system_run_enabled = false;
static bool wakeup_requested = false;

// Flag um den Discovery-Timeout selbst zu managen
static bool discovery_waiting = false;
static uint32_t discovery_send_tick = 0;

/* ================================================================================== */
/*                             MODBUS CALLBACKS                                       */
/* ================================================================================== */

static void OnMasterComplete(void) {
    /* --- Discovery Phase --- */
    if (current_state == AS_DISCOVER_WAIT) {
        // Antwort erhalten! Dieser Slave existiert!
        if (slave_count < ARBITER_MAX_SLAVES) {
            active_slaves[slave_count] = discovery_current_id;
            slave_states[slave_count].wunsch    = register_map[0];
            slave_states[slave_count].emergency = register_map[1];
            slave_states[slave_count].node_type = register_map[2];
            slave_count++;

            const char *type_str = "???";
            if (register_map[2] == NODE_TYPE_INVERTER) type_str = "INV";
            else if (register_map[2] == NODE_TYPE_DCDC) type_str = "DCDC";

            snprintf(itm_buf, sizeof(itm_buf), "[DISC] Found Slave ID %d -> Type: %s\n", discovery_current_id, type_str);
            ITM_Print(itm_buf);
        }
        discovery_waiting = false;
        discovery_current_id++;
        if (discovery_current_id > DISCOVERY_MAX_ID) {
            snprintf(itm_buf, sizeof(itm_buf), "[DISC] Complete. %d slaves found.\n", slave_count);
            ITM_Print(itm_buf);
            target_idx = 0;
            current_state = AS_DELAY;
            delay_tick = HAL_GetTick();
        } else {
            current_state = AS_DISCOVER;
        }
        return;
    }

    /* --- Normaler Polling-Betrieb --- */
    if (current_state == AS_WAIT_RX) {
        slave_states[target_idx].wunsch    = register_map[0];
        slave_states[target_idx].emergency = register_map[1];
        slave_states[target_idx].node_type = register_map[2];

        target_idx++;
        if (target_idx >= slave_count) {
            current_state = AS_EVALUATE;
        } else {
            current_state = AS_POLL_RX;
        }
    }
    else if (current_state == AS_WAIT_TX) {
        target_idx++;
        if (target_idx >= slave_count) {
            current_state = AS_DELAY;
            delay_tick = HAL_GetTick();
        } else {
            current_state = AS_POLL_TX;
        }
    }
}

static void OnMasterError(Modbus_Error_t error) {
    /* --- Während Discovery: Timeout = ID nicht besetzt, weiter machen --- */
    if (current_state == AS_DISCOVER_WAIT) {
        discovery_waiting = false;
        discovery_current_id++;
        if (discovery_current_id > DISCOVERY_MAX_ID) {
            snprintf(itm_buf, sizeof(itm_buf), "[DISC] Complete. %d slaves found.\n", slave_count);
            ITM_Print(itm_buf);
            target_idx = 0;
            current_state = AS_DELAY;
            delay_tick = HAL_GetTick();
        } else {
            current_state = AS_DISCOVER;
        }
        return;
    }

    /* --- Normalbetrieb: Fehler = Notaus --- */
    current_state = AS_ERROR;
}

/* ================================================================================== */
/*                                PUBLIC API                                          */
/* ================================================================================== */

void Arbiter_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim) {
    slave_count = 0;
    memset(active_slaves, 0, sizeof(active_slaves));
    memset(slave_states, 0, sizeof(slave_states));

    Modbus_Init(&hmodbus, huart, htim, 0); // Master hat ID 0
    hmodbus.master_complete_cb = OnMasterComplete;
    hmodbus.error_cb = OnMasterError;

    // Discovery beginnt bei ID 1
    discovery_current_id = 1;
    current_state = AS_DISCOVER;

    ITM_Print("[ARBITER] System booting. Starting Auto-Discovery...\n");
}

void Arbiter_SetGlobalRunState(bool run) {
    system_run_enabled = run;
    if (run) {
        ITM_Print("[ARBITER] System ENABLED by operator.\n");
    } else {
        ITM_Print("[ARBITER] System DISABLED by operator (manual stop).\n");
    }
}

bool Arbiter_GetGlobalRunState(void) {
    return system_run_enabled;
}

uint16_t Arbiter_GetSystemMode(void) {
    return global_allowed_mode;
}

void Arbiter_TriggerWakeup(void) {
    wakeup_requested = true;
}

/* ================================================================================== */
/*                            ARBITER STATE MACHINE                                   */
/* ================================================================================== */

void Arbiter_Process(void) {
    Modbus_Master_Process(&hmodbus);

    switch (current_state) {

        /* ============================================================ */
        /*                    AUTO-DISCOVERY PHASE                      */
        /* ============================================================ */
        case AS_DISCOVER:
            // Sende FC03 an die aktuelle Discovery-ID (3 Register: Wunsch, Emergency, NodeType)
            Modbus_Master_Request(&hmodbus, discovery_current_id, MB_FC_READ_HOLDING_REGISTERS, REG_WUNSCH, 3);
            discovery_waiting = true;
            discovery_send_tick = HAL_GetTick();
            current_state = AS_DISCOVER_WAIT;
            break;

        case AS_DISCOVER_WAIT:
            // Der normale Modbus-Timeout (1000ms) ist für Discovery zu langsam.
            // Wir managen hier einen eigenen, schnelleren Timeout.
            if (discovery_waiting && (HAL_GetTick() - discovery_send_tick > DISCOVERY_TIMEOUT_MS)) {
                // Manuell abbrechen und weitermachen
                hmodbus.state = MB_STATE_IDLE;
                hmodbus.frame_complete = false;
                discovery_waiting = false;
                discovery_current_id++;
                if (discovery_current_id > DISCOVERY_MAX_ID) {
                    snprintf(itm_buf, sizeof(itm_buf), "[DISC] Complete. %d slaves found.\n", slave_count);
                    ITM_Print(itm_buf);
                    target_idx = 0;
                    current_state = AS_DELAY;
                    delay_tick = HAL_GetTick();
                } else {
                    current_state = AS_DISCOVER;
                }
            }
            break;

        /* ============================================================ */
        /*                    NORMALER POLLING-BETRIEB                  */
        /* ============================================================ */
        case AS_POLL_RX:
            if (slave_count > 0) {
                // Lese 3 Register: Wunsch, Emergency, NodeType
                Modbus_Master_Request(&hmodbus, active_slaves[target_idx], MB_FC_READ_HOLDING_REGISTERS, REG_WUNSCH, 3);
                current_state = AS_WAIT_RX;
            } else {
                // Kein Slave gefunden, im Delay bleiben
                current_state = AS_DELAY;
                delay_tick = HAL_GetTick();
            }
            break;

        case AS_WAIT_RX:
            // Wartet auf OnMasterComplete
            break;

        /* ============================================================ */
        /*                 INTELLIGENTES VOTING                         */
        /* ============================================================ */
        case AS_EVALUATE:
            {
                bool emergency_found = false;
                int8_t inverter_idx = -1; // Index des Inverters im Array (-1 = keiner)

                // --- Pass 1: Emergency checken & Inverter identifizieren ---
                for (uint8_t i = 0; i < slave_count; i++) {
                    if (slave_states[i].emergency > 0) {
                        emergency_found = true;
                    }
                    if (slave_states[i].node_type == NODE_TYPE_INVERTER) {
                        inverter_idx = i;
                    }
                }

                // --- Pass 2: Entscheidung treffen ---
                if (emergency_found || !system_run_enabled) {
                    global_allowed_mode = SYSTEM_MODE_STANDBY;
                } else if (wakeup_requested) {
                    global_allowed_mode = SYSTEM_MODE_WAKEUP;
                    wakeup_requested = false;
                } else if (inverter_idx >= 0) {
                    // INVERTER-PRIORITÄT: Der Wunsch des Inverters bestimmt das System!
                    global_allowed_mode = slave_states[inverter_idx].wunsch;
                } else {
                    // Kein Inverter gefunden (nur DC/DCs): Einstimmigkeit nötig
                    uint16_t first = slave_states[0].wunsch;
                    bool all_agree = true;
                    for (uint8_t i = 1; i < slave_count; i++) {
                        if (slave_states[i].wunsch != first) {
                            all_agree = false;
                            break;
                        }
                    }
                    global_allowed_mode = all_agree ? first : SYSTEM_MODE_STANDBY;
                }

                // --- ITM Dashboard Output ---
                const char *mode_str = "STANDBY";
                if (global_allowed_mode == SYSTEM_MODE_CHARGE) mode_str = "CHARGE";
                else if (global_allowed_mode == SYSTEM_MODE_DISCHARGE) mode_str = "DISCHARGE";
                else if (global_allowed_mode == SYSTEM_MODE_WAKEUP) mode_str = "WAKEUP";

                snprintf(itm_buf, sizeof(itm_buf),
                    "[VOTE] Run:%s | Mode:%s | Slaves:%d",
                    system_run_enabled ? "ON" : "OFF",
                    mode_str,
                    slave_count);
                ITM_Print(itm_buf);

                // Slave-Details
                for (uint8_t i = 0; i < slave_count; i++) {
                    const char *t = (slave_states[i].node_type == NODE_TYPE_INVERTER) ? "INV" : "DCDC";
                    snprintf(itm_buf, sizeof(itm_buf), " | [%d]%s:W=%d", active_slaves[i], t, slave_states[i].wunsch);
                    ITM_Print(itm_buf);
                }
                ITM_Print("\n");

                target_idx = 0;
                current_state = AS_POLL_TX;
            }
            break;

        case AS_POLL_TX:
            if (slave_count > 0) {
                Modbus_Master_Request(&hmodbus, active_slaves[target_idx], MB_FC_WRITE_SINGLE_REGISTER, REG_ERLAUBNIS, global_allowed_mode);
                current_state = AS_WAIT_TX;
            }
            break;

        case AS_WAIT_TX:
            // Wartet auf OnMasterComplete
            break;

        case AS_DELAY:
            if (HAL_GetTick() - delay_tick > 100) {
                target_idx = 0;
                current_state = AS_POLL_RX;
            }
            break;

        case AS_ERROR:
            {
                snprintf(itm_buf, sizeof(itm_buf), "[ERROR] Communication failure! Safety stop triggered.\n");
                ITM_Print(itm_buf);

                HAL_Delay(50);
                Modbus_Master_Request(&hmodbus, 0, MB_FC_WRITE_SINGLE_REGISTER, REG_SAFETY_STOP, 1);

                global_allowed_mode = SYSTEM_MODE_STANDBY;
                delay_tick = HAL_GetTick();
                target_idx = 0;
                current_state = AS_DELAY;
            }
            break;
    }
}
