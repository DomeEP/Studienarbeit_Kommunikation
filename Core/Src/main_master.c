/**
  * @file    main_master.c (MASTER - T3100 ARBITER)
  * @brief   Polling-basierter Modbus-Master mit Voting-Logik (Arbiter)
  *
  * Der Master pollt zyklisch alle Slaves, wertet deren "Wunsch"-Register
  * aus und sendet den erlaubten Systemstatus zurück. Bei Fehlern wird
  * automatisch ein Safety-Stop ausgelöst.
  */
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "modbus_rtu.h"
#include "app_config.h"

Modbus_Handle_t hmodbus;

void SystemClock_Config(void);

/* --- LED Hilfsfunktionen --- */
static void led_on(void)  { HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET); }
static void led_off(void) { HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET); }

uint16_t register_map[APP_REGISTER_MAP_SIZE] = {0};

/* --- Arbiter States --- */
#define ARBITER_STATE_POLL_SLAVE   0
#define ARBITER_STATE_WAIT_POLL    1
#define ARBITER_STATE_WRITE_SLAVE  2
#define ARBITER_STATE_WAIT_WRITE   3
#define ARBITER_STATE_DELAY        4
#define ARBITER_STATE_ERROR        99

volatile uint8_t arbiter_state = ARBITER_STATE_POLL_SLAVE;
uint32_t arbiter_delay_tick = 0;
uint16_t current_allowed_mode = MODE_STANDBY;

/* --- Modbus Callbacks --- */
void OnMasterComplete(void) {
    if (arbiter_state == ARBITER_STATE_WAIT_POLL) {
        arbiter_state = ARBITER_STATE_WRITE_SLAVE;
    } else if (arbiter_state == ARBITER_STATE_WAIT_WRITE) {
        arbiter_state = ARBITER_STATE_DELAY;
    }
}

void OnMasterError(Modbus_Error_t error) {
    // Bei jedem Fehler sofort in den Notaus-Write-Cycle springen
    arbiter_state = ARBITER_STATE_ERROR;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* UART Interrupts an */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* Modbus initialisieren */
  Modbus_Init(&hmodbus, &huart2, &htim6, 0);
  hmodbus.master_complete_cb = OnMasterComplete;
  hmodbus.error_cb = OnMasterError;

  while (1)
  {
      Modbus_Master_Process(&hmodbus);

      switch (arbiter_state) {
          case ARBITER_STATE_POLL_SLAVE:
              // Lese Wunsch (Reg 0) und Emergency (Reg 1) von Inverter (ID=1)
              Modbus_Master_Request(&hmodbus, 1, MB_FC_READ_HOLDING_REGISTERS, REG_WUNSCH, 2);
              arbiter_state = ARBITER_STATE_WAIT_POLL;
              break;

          case ARBITER_STATE_WAIT_POLL:
              // Warten auf Callback...
              break;

          case ARBITER_STATE_WRITE_SLAVE:
              // Evaluate Logic
              if (register_map[1] == 1) { 
                  // Emergency aktiv!
                  current_allowed_mode = MODE_STANDBY;
                  led_off();
              } else {
                  // In diesem Setup mit nur 1 Slave diktiert der Inverter-Wunsch das System.
                  // Bei mehreren Slaves würden wir hier mit && Verknüpfen prüfen.
                  current_allowed_mode = register_map[0]; 
                  
                  if (current_allowed_mode == MODE_CHARGE) led_on();
                  else led_off(); // LED aus für Standby / blinkt beim Slave für Discharge
              }

              // Schreibe Erlaubnis (Reg 10) zurück
              Modbus_Master_Request(&hmodbus, 1, MB_FC_WRITE_SINGLE_REGISTER, REG_ERLAUBNIS, current_allowed_mode);
              arbiter_state = ARBITER_STATE_WAIT_WRITE;
              break;

          case ARBITER_STATE_WAIT_WRITE:
              // Warten auf Callback...
              break;

          case ARBITER_STATE_DELAY:
              arbiter_delay_tick = HAL_GetTick();
              arbiter_state = 5; // Delay state
              break;
              
          case 5:
              if (HAL_GetTick() - arbiter_delay_tick > 100) { // 10 Hz Polling
                  arbiter_state = ARBITER_STATE_POLL_SLAVE;
              }
              break;

          case ARBITER_STATE_ERROR:
              // Sende harten Emergency Stop an alle (oder an Inverter) wenn Timeout
              // Warte kurz damit der Bus frei wird
              HAL_Delay(50);
              Modbus_Master_Request(&hmodbus, 1, MB_FC_WRITE_SINGLE_REGISTER, REG_SAFETY_STOP, 1);
              current_allowed_mode = MODE_STANDBY;
              led_off();
              arbiter_delay_tick = HAL_GetTick();
              arbiter_state = 5; // Gehe danach in Delay
              break;
      }
  }
} // End of main()

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; o.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&o) != HAL_OK) { Error_Handler(); }
  c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV1; c.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}
void Error_Handler(void) { __disable_irq(); while(1){} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) { (void)f;(void)l; }
#endif
