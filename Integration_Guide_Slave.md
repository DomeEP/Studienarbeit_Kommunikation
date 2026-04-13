# T3100 Modbus RTU – Integrations-Anleitung (Slave)

Diese Anleitung beschreibt, wie du den Modbus-Kommunikations-Stack in dein STM32G4-Projekt integrierst, um als Slave-Knoten (Inverter oder DC/DC-Stufe) am Gesamt-System teilzunehmen.

---

## 1. Dateien kopieren
Kopiere die folgenden Verzeichnisse aus diesem Projekt in dein eigenes Projektverzeichnis (meistens unter `Drivers/`):

*   `Drivers/Modbus/` (Enthält das core Modbus-Protokoll)
*   `Drivers/SlaveNode/` (Enthält die vereinfachte API für Fachgruppen)

---

## 2. Hardware-Konfiguration (STM32CubeMX)

Der Modbus-Stack benötigt einen UART für die physikalische Übertragung und einen Timer für die Rahmenerkennung.

### RS485-Schnittstelle (Connectivity -> USARTx)
1.  **Mode**: `Asynchronous`
2.  **Hardware Flow Control (RS485)**: `Enable` (**WICHTIG!** Steuert den DE-Pin des Transceivers)
3.  **Baudrate**: `115200` Bits/s
4.  **Word Length**: `8 Bits`
5.  **Stop Bits**: `1`
6.  **NVIC Settings**: Aktiviere den `USARTx global interrupt`.

### Zeitgeber (Internal Timers -> TIMx, z.B. TIM6)
1.  **Activated**: Checkbox oder `Internal Clock` wählen.
2.  **Konfiguration**: Die Werte für Prescaler und Period sind egal, da der Treiber den Timer beim Start automatisch auf die korrekte Modbus-Zeit (1,75 ms) kalibriert.
3.  **NVIC Settings**: Aktiviere den `TIMx global interrupt`.

### Interrupt-Prioritäten (System Core -> NVIC)
Damit das Timing exakt eingehalten wird, müssen die Prioritäten korrekt gesetzt sein:
*   **TIMx global interrupt**: Priorität **0** (höchste)
*   **USARTx global interrupt**: Priorität **1** (oder niedriger als der Timer)

---

## 3. Code-Integration

### 3.1 Interrupts verbinden (`stm32g4xx_it.c`)
Du musst die Interrupt-Handler in der Datei `stm32g4xx_it.c` mit dem Treiber verknüpfen.

```c
/* Header-Bereich */
#include "modbus_rtu.h"
extern Modbus_Handle_t hmodbus; // Das Handle wird im SlaveNode-Treiber deklariert

/* In der USARTx_IRQHandler Funktion */
void USART2_IRQHandler(void) {
  Modbus_IRQHandler_RxCplt(&hmodbus);
  // Falls CubeMX eigenen Code generiert hat, diesen nach den Modbus-Aufrufen lassen
}

/* In der TIMx_IRQHandler Funktion */
void TIM6_DAC_IRQHandler(void) {
  Modbus_IRQHandler_Timeout(&hmodbus);
  HAL_TIM_IRQHandler(&htim6); // Standard HAL Aufruf
}
```

### 3.2 Initialisierung (`main.c`)
In deiner `main.c` startest du die Kommunikation:

```c
/* Header */
#include "slave_node.h"

/* Vor der main-Schleife */
// Die ID muss eindeutig sein: 1=Inverter, 2=DCDC_1, 3=DCDC_2, etc.
SlaveNode_Init(&huart2, &htim6, 2, NULL); 
SlaveNode_SetNodeType(NODE_TYPE_DCDC); // Deinen Knotentyp festlegen

/* In der while(1) Schleife */
while (1) {
    SlaveNode_Process(); // Hält den Stack am Leben
    
    // Dein Applikationscode...
}
```

---

## 4. Verwendung der API

Der Treiber kapselt die Modbus-Register, damit du dich nicht um Register-Adressen kümmern musst.

### Wunsch äußern
Teile dem Master mit, in welchen Modus du gerne gehen würdest (z.B. basierend auf einem Tastendruck oder Sensorsignal).
```c
SlaveNode_SetDesiredMode(SYSTEM_MODE_CHARGE);
```

### Erlaubnis prüfen (**WICHTIG!**)
Deine Hardware darf nur schalten, wenn der Master die Erlaubnis dazu gibt. Prüfe dies zyklisch:
```c
uint16_t allowed = SlaveNode_GetAllowedMode();

if (allowed == SYSTEM_MODE_CHARGE) {
    // Schütze schließen, Strom regeln...
} else {
    // Alles sicher trennen / Standby
}
```

### Notaus melden
Falls deine Stufe einen kritischen Fehler erkennt (z.B. Übertemperatur oder Überstrom), setzt du das Emergency-Flag. Der Master wird daraufhin das gesamte System abschalten.
```c
SlaveNode_SetEmergencyFlag(true);
```

---

## 5. Modbus Register Map (Referenz)

Falls du manuell auf Register zugreifen willst (nur für Fortgeschrittene):

| Adresse | Typ | Name | Beschreibung |
| :--- | :--- | :--- | :--- |
| **0** | R | `REG_WUNSCH` | Dein lokaler Wunschmodus (0=Standby, 1=Charge, 2=Disch.) |
| **1** | R | `REG_EMERGENCY`| Dein lokales Fehler-Flag (0=OK, 1=Kritischer Fehler) |
| **2** | R | `REG_NODE_TYPE`| Typ deiner Hardware (1=Inverter, 2=DC/DC) |
| **10** | W | `REG_ERLAUBNIS`| Vom Master genehmigter Modus (0, 1, 2) |
| **11** | W | `REG_SAFETY_STOP`| Systemweiter Notstopp-Befehl vom Master |

> [!TIP]
> **Tipp für Debugging**: In der Struktur `hmodbus.stats` findest du Zähler für empfangene Frames und Fehler. Das hilft bei der Fehlersuche am RS485-Bus.
