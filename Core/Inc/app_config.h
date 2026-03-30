/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Zentrale Konfiguration für die Studienarbeit Kommunikation
  ******************************************************************************
  * @details
  * Diese Datei enthält ALLE anpassbaren Parameter der Firmware an einem Ort.
  * Beim Erweitern des Systems (neue Register, neue Slaves, andere Baudraten)
  * muss nur diese Datei geändert werden.
  *
  * HARDWARE-ÜBERSICHT (laut Schaltplan):
  * ┌─────────────────────────────────────────────────────────┐
  * │  WeActStudio STM32G474 CoreBoard                        │
  * │                                                         │
  * │  Interne Peripherie:                                    │
  * │    PC13  = BTN_intern  (User-Button, Active-LOW)        │
  * │    PC6   = LED_intern  (Onboard-LED)                    │
  * │                                                         │
  * │  Externe Peripherie (auf Masterboard-PCB):              │
  * │    PB6   = LED1        (Externe Status-LED)             │
  * │    PB7   = LED2        (Externe Status-LED)             │
  * │    PB9   = BTN         (Externer Button)                │
  * │                                                         │
  * │  RS485 (ADM2587E Transceiver):                          │
  * │    PA1   = USART2_DE   (Driver Enable, Active-HIGH)     │
  * │    PA2   = USART2_TX   (Transmit Data)                  │
  * │    PA3   = USART2_RX   (Receive Data)                   │
  * │                                                         │
  * │  I2C (für Sensoren/Displays):                           │
  * │    PA8   = I2C2_SDA    (Datenleitung)                   │
  * │    PA9   = I2C2_SCL    (Taktleitung)                    │
  * │                                                         │
  * │  Stromversorgung:                                       │
  * │    USB-C       → 5V/3.3V für STM32 + ADM2587E Logik    │
  * │    J1 (12V)    → TBA1-1211 DC-DC → 5V isoliert          │
  * │                  → ADM2587E VISOIN (RS485 Busseite)     │
  * │    HINWEIS: Ohne 12V an J1 ist der RS485-Bus tot!       │
  * └─────────────────────────────────────────────────────────┘
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================================
 *  MODBUS KONFIGURATION
 * ============================================================================ */

/** @brief Modbus Slave-Adresse dieses Geräts (nur relevant im Slave-Modus)
 *  Gültiger Bereich: 1-247. Adresse 0 = Broadcast (alle Slaves). */
#define APP_SLAVE_ID              1

/** @brief RS485 Baudrate (muss auf allen Teilnehmern gleich sein!)
 *  Übliche Werte: 9600, 19200, 38400, 57600, 115200 */
#define APP_BAUDRATE              115200

/** @brief Timeout für Slave-Antwort in Millisekunden (Master-Modus)
 *  Wenn der Slave nicht innerhalb dieser Zeit antwortet, gilt die
 *  Anfrage als fehlgeschlagen. */
#define APP_RESPONSE_TIMEOUT_MS   1000

/* ============================================================================
 *  REGISTER-MAP DEFINITION (Bidirektionaler Test)
 * ============================================================================
 *
 * Adresse 0x0000 = REG_MASTER_CMD
 *   Der Master schreibt hier rein, wie lange die Slave-LED leuchten soll.
 *   (z.B. 3000 für 3 Sekunden wenn der Master-Button gedrückt wird)
 *
 * Adresse 0x0001 = REG_SLAVE_BTN_CNT
 *   Der Slave erhöht diesen Zähler bei jedem eigenen Tastendruck.
 *   Der Master liest (polled) dieses Register ständig um den Druck zu erkennen.
 */

#define APP_REGISTER_MAP_SIZE     2

#define REG_MASTER_CMD            0x0000  /**< Master sendet: LED-Leuchtdauer (ms) */
#define REG_SLAVE_BTN_CNT         0x0001  /**< Slave sendet: Tastenzähler */

/* ============================================================================
 *  TIMING KONFIGURATION
 * ============================================================================ */

/** @brief Heartbeat-Intervall in Millisekunden (LED blinkt mit dieser Rate) */
#define APP_HEARTBEAT_INTERVAL_MS 500

/** @brief Entprellzeit für Buttons in Millisekunden */
#define APP_DEBOUNCE_DELAY_MS     50

/* ============================================================================
 *  ERWEITERUNGS-HINWEISE
 * ============================================================================
 *
 * 1. NEUEN SLAVE HINZUFÜGEN:
 *    - Neue Slave-ID in APP_SLAVE_ID ändern (jeder Slave braucht eine eigene!)
 *    - Im Master-Code die Ziel-ID beim Senden anpassen
 *
 * 2. NEUES REGISTER HINZUFÜGEN:
 *    - APP_REGISTER_MAP_SIZE erhöhen
 *    - Neues #define REG_xxx mit der nächsten Adresse anlegen
 *    - In main_slave.c → App_Slave_OnRegisterWrite() die Logik ergänzen
 *
 * 3. SENSORDATEN SENDEN:
 *    - Im Slave: Sensorwert in ein Register schreiben (z.B. register_map[REG_xxx] = wert)
 *    - Im Master: Mit Modbus_Master_Request(FC03, REG_xxx, 1) das Register auslesen
 *
 * 4. MEHRERE SLAVES:
 *    - Jeder Slave bekommt eine eigene ID (1, 2, 3, ...)
 *    - Der Master adressiert jeden einzeln über den slave_id Parameter
 */

#endif /* APP_CONFIG_H */
