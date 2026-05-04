# T3200 Modbus RTU – Integrations-Anleitung (Slave)

Diese Anleitung beschreibt, wie der Modbus-Kommunikations-Stack in ein bestehendes STM32G4-Projekt integriert wird, um als Slave-Knoten (Inverter oder DC/DC-Stufe) am Microgrid teilzunehmen.

---

## 1. Dateien kopieren

Folgende Verzeichnisse in das eigene Projekt übernehmen (typischerweise unter `Drivers/`):

*   `Drivers/Modbus/` – Protokoll-Kern (modbus_rtu.h/.c)
*   `Drivers/SlaveNode/` – High-Level API (slave_node.h/.c)

Zusätzlich die zentrale Konfigurationsdatei `Core/Inc/app_config.h` kopieren und an das eigene Projekt anpassen.

---

## 2. Hardware-Konfiguration (STM32CubeMX)

### RS485-Schnittstelle (Connectivity → USARTx)
1.  **Mode**: `Asynchronous`
2.  **Hardware Flow Control (RS485)**: `Enable` – steuert den DE-Pin des ADM2587E
3.  **Baudrate**: `115200` Bits/s
4.  **Word Length**: `8 Bits`, **Stop Bits**: `1`
5.  **NVIC Settings**: `USARTx global interrupt` aktivieren

### Zeitgeber für Rahmenerkennung (Timers → TIMx, z.B. TIM6)
1.  `Internal Clock` aktivieren
2.  Prescaler und Period können beliebig gesetzt werden – der Treiber kalibriert den Timer beim Start automatisch auf 1,75 ms (Modbus T3.5)
3.  **NVIC Settings**: `TIMx global interrupt` aktivieren

### Interrupt-Prioritäten (System Core → NVIC)
Die Timer-Priorität muss höher sein als die UART-Priorität:

| Interrupt              | Priorität |
|------------------------|-----------|
| TIMx global interrupt  | 0         |
| USARTx global interrupt| 1         |

---

## 3. Code-Integration

### 3.1 Interrupts verbinden (`stm32g4xx_it.c`)

```c
/* --- Includes --- */
#include "modbus_rtu.h"
extern Modbus_Handle_t hmodbus;

/* --- USART Interrupt --- */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

/* --- Timer Interrupt --- */
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}

/* --- HAL Callbacks (am Ende der Datei) --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        Modbus_IRQHandler_RxCplt(&hmodbus);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        Modbus_IRQHandler_Timeout(&hmodbus);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        Modbus_IRQHandler_Error(&hmodbus);
    }
}
```

### 3.2 Initialisierung (`main.c`)

```c
#include "slave_node.h"

/* Optional: Callback bei Master-Notaus */
void OnEmergency(void) {
    // Leistungselektronik trennen, LED aus, etc.
}

int main(void) {
    /* ... HAL_Init, Clock, Peripherie ... */

    /* Kommunikation starten.
     * ID muss pro Knoten eindeutig sein: 1=Inverter, 2=DCDC_1, 3=DCDC_2, ...
     */
    SlaveNode_Init(&huart2, &htim6, 2, OnEmergency);
    SlaveNode_SetNodeType(NODE_TYPE_DCDC);

    while (1) {
        SlaveNode_Process();   // Modbus-Stack antreiben

        // --- Eigene Applikationslogik ---
    }
}
```

---

## 4. API-Referenz

### Wunschmodus setzen
```c
SlaveNode_SetDesiredMode(SYSTEM_MODE_CHARGE);
```

### Master-Erlaubnis prüfen
Die eigene Hardware darf ausschließlich auf Basis der Master-Erlaubnis schalten:
```c
uint16_t allowed = SlaveNode_GetAllowedMode();

if (allowed == SYSTEM_MODE_CHARGE) {
    // Schütze schließen, Strom regeln
} else {
    // Standby, alles sicher trennen
}
```

### Notaus melden
Bei kritischen Fehlern (Übertemperatur, Überstrom) setzt das Emergency-Flag den gesamten Bus in den Standby:
```c
SlaveNode_SetEmergencyFlag(true);
```

---

## 5. Modbus Register Map

| Adresse | Richtung       | Name              | Beschreibung                                      |
|---------|----------------|-------------------|---------------------------------------------------|
| 0       | Slave → Master | `REG_WUNSCH`      | Lokaler Wunschmodus (0=Standby, 1=Charge, 2=Disch.) |
| 1       | Slave → Master | `REG_EMERGENCY`   | Fehler-Flag (0=OK, 1=Kritisch)                    |
| 2       | Slave → Master | `REG_NODE_TYPE`   | Hardware-Typ (1=Inverter, 2=DC/DC)                |
| 10      | Master → Slave | `REG_ERLAUBNIS`   | Vom Master genehmigter Betriebsmodus               |
| 11      | Master → Slave | `REG_SAFETY_STOP` | Systemweiter Notstopp-Befehl                       |

---

## 6. Build (falls das Gesamtprojekt gebaut wird)

```bash
# Slave-Firmware
cmake -B build_slave -DBOARD_ROLE=slave -DUSE_MODBUS=ON --preset default
cmake --build build_slave

# Master-Firmware
cmake -B build_master -DBOARD_ROLE=master -DUSE_MODBUS=ON --preset default
cmake --build build_master
```

---

## 7. Debugging-Tipps

- Die Struktur `hmodbus.stats` enthält Zähler für empfangene Frames (`rx_frames`), CRC-Fehler (`crc_errors`) und Timeouts – hilfreich bei Bus-Problemen.
- Die Master-Telemetrie kann über USB (virtueller COM-Port, 115200 Baud) in PuTTY mitgelesen werden.
- Falls der Bus nicht reagiert: Prüfen ob 12 V an J1 anliegen (ADM2587E-Versorgung).
