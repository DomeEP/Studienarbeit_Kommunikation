# T3200 – Modbus RTU Kommunikations-Stack

Firmware für die zentrale Kommunikationsinfrastruktur des T3200-Netzstabilisierungssystems.  
Implementiert auf dem **STM32G474** (WeAct CoreBoard) mit galvanisch getrenntem **RS485**-Bus (ADM2587E).

---

## Projektstruktur

```
├── Core/
│   ├── Inc/
│   │   └── app_config.h          ← Zentrale Konfiguration (IDs, Baudrate, Register)
│   └── Src/
│       ├── main_master.c         ← Einstiegspunkt: Master-Firmware (Arbiter)
│       ├── main_slave.c          ← Einstiegspunkt: Slave-Firmware (Fachgruppen)
│       └── stm32g4xx_it.c        ← Interrupt-Routing (UART → Modbus, TIM → T3.5)
│
├── Drivers/
│   ├── Modbus/
│   │   ├── modbus_rtu.h/.c       ← Protokoll-Kern: CRC16, State Machine, Frame-Erkennung
│   │   └── modbus_test.c         ← Offline-Selbsttest (CRC + Frame-Logik)
│   ├── SlaveNode/
│   │   └── slave_node.h/.c       ← High-Level API für Slave-Knoten
│   └── Arbiter/
│       └── system_arbiter.h/.c   ← Master-Logik: Discovery, Voting, Telemetrie
│
├── Integration_Guide_Slave.md    ← Schritt-für-Schritt-Anleitung für Fachgruppen
├── CMakeLists.txt                ← Build-System (Master/Slave über BOARD_ROLE)
└── CubeMX.ioc                   ← STM32CubeMX Projekt-Datei
```

---

## Architektur

```
┌──────────────────────────────────────────────────────────┐
│  Applikation    main_master.c / main_slave.c             │
├──────────────────────────────────────────────────────────┤
│  Abstraktion    system_arbiter.c    │   slave_node.c     │
│                 (Master-Seite)      │   (Slave-Seite)    │
├─────────────────────────────────────┴────────────────────┤
│  Protokoll      modbus_rtu.c  (gemeinsamer Kern)         │
├──────────────────────────────────────────────────────────┤
│  Hardware       USART2 + TIM6 + ADM2587E (RS485)         │
└──────────────────────────────────────────────────────────┘
```

**Master** (Arbiter): Führt nach dem Booten eine Auto-Discovery durch (Adressen 1–10),
pollt zyklisch alle gefundenen Slaves, berechnet per Voting-Algorithmus den globalen
Betriebsmodus und schreibt diesen an alle Teilnehmer zurück.  
Live-Telemetrie wird über USB CDC an ein Terminal (PuTTY) ausgegeben.

**Slave** (Knoten): Beantwortet Modbus-Anfragen mit seinem lokalen Wunschmodus,
Knotentyp und Emergency-Status. Schaltet seine Hardware ausschließlich auf Basis
der vom Master erteilten Erlaubnis.

---

## Build

Voraussetzung: `arm-none-eabi-gcc` Toolchain + CMake ≥ 3.22.

```bash
# Master-Firmware bauen
cmake -B build_master -DBOARD_ROLE=master -DUSE_MODBUS=ON --preset default
cmake --build build_master

# Slave-Firmware bauen
cmake -B build_slave -DBOARD_ROLE=slave -DUSE_MODBUS=ON --preset default
cmake --build build_slave
```

Die erzeugten `.bin`- und `.hex`-Dateien liegen im jeweiligen Build-Ordner.

### Flashen (STM32CubeProgrammer oder ST-Link CLI)

```bash
STM32_Programmer_CLI -c port=SWD -w build_master/Studienarbeit_Kommunikation.bin 0x08000000
```

---

## Hardware-Pinout (WeAct STM32G474 CoreBoard)

| Pin   | Funktion         | Beschreibung                                |
|-------|------------------|---------------------------------------------|
| PA1   | USART2_DE        | RS485 Driver Enable (Hardware Flow Control) |
| PA2   | USART2_TX        | RS485 Sendedaten                            |
| PA3   | USART2_RX        | RS485 Empfangsdaten                         |
| PA8   | I2C2_SDA         | I²C Datenleitung (Erweiterungsport)         |
| PA9   | I2C2_SCL         | I²C Taktleitung (Erweiterungsport)          |
| PA13  | SWDIO            | Programmierung (ST-Link)                    |
| PA14  | SWCLK            | Programmierung (ST-Link)                    |
| PB6   | LED1             | Externe Status-LED auf Masterboard          |
| PB7   | LED2             | Externe Status-LED auf Masterboard          |
| PB9   | BTN              | Externer Taster auf Masterboard             |
| PC6   | LED_intern       | Onboard-LED (CoreBoard)                     |
| PC13  | BTN_intern       | Onboard-Button (CoreBoard)                  |

**Wichtig:** Der RS485-Bus ist nur aktiv, wenn 12 V an der externen Versorgungsbuchse (J1) anliegen.  
Ohne diese Spannung hat der ADM2587E-Transceiver keine Bus-Versorgung.

---

## Modbus Register Map

| Adresse | Richtung       | Name             | Werte                                      |
|---------|----------------|------------------|---------------------------------------------|
| 0       | Slave → Master | `REG_WUNSCH`     | 0 = Standby, 1 = Charge, 2 = Discharge     |
| 1       | Slave → Master | `REG_EMERGENCY`  | 0 = OK, 1 = Kritischer Fehler              |
| 2       | Slave → Master | `REG_NODE_TYPE`  | 1 = Inverter, 2 = DC/DC                    |
| 10      | Master → Slave | `REG_ERLAUBNIS`  | 0 = Standby, 1 = Charge, 2 = Discharge     |
| 11      | Master → Slave | `REG_SAFETY_STOP`| 1 = Systemweiter Notstopp                  |

---

## Für die Nachfolgegruppe

Die Integration als Slave-Knoten ist in der [Integrations-Anleitung](Integration_Guide_Slave.md) beschrieben.  
Kurzfassung:

1. `Drivers/Modbus/` und `Drivers/SlaveNode/` in euer Projekt kopieren
2. UART + Timer in CubeMX konfigurieren (siehe Guide)
3. Interrupts in `stm32g4xx_it.c` verdrahten
4. In `main.c`: `SlaveNode_Init()` aufrufen, `SlaveNode_Process()` in der Hauptschleife

Die komplette API ist in [`slave_node.h`](Drivers/SlaveNode/slave_node.h) dokumentiert.
