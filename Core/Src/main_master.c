/* USER CODE BEGIN Header */
/**
  * @file    main.c (MASTER - BIDIREKTIONALER MODBUS)
  * @brief   Implementiert die Master-Logik für Buttons & LEDs via FC03/FC06
  */
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "modbus_rtu.h"
#include "app_config.h"

Modbus_Handle_t hmodbus;

typedef enum {
    MSTATE_IDLE,
    MSTATE_POLLING_SLAVE,
    MSTATE_SENDING_CMD
} MasterState_t;
MasterState_t master_state = MSTATE_IDLE;

/* --- Timers & State --- */
uint32_t led_off_tick = 0;
uint8_t led_is_on = 0;

uint32_t last_btn_tick = 0;
GPIO_PinState last_btn_state = GPIO_PIN_SET;

uint32_t last_poll_tick = 0;
uint16_t last_slave_cnt = 0;
uint8_t first_poll_done = 0;

void SystemClock_Config(void);

/* --- Modbus Callbacks --- */
void OnMasterComplete(void) {
    if (master_state == MSTATE_POLLING_SLAVE) {
        uint16_t new_cnt = (hmodbus.rx_buffer[3] << 8) | hmodbus.rx_buffer[4];
        
        if (first_poll_done && (new_cnt != last_slave_cnt)) {
            /* Slave-Zähler hat sich erhöht -> Slave Button wurde gedrückt! -> 1s leuchten */
            HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET); // LED AN (Active-HIGH)
            led_off_tick = HAL_GetTick() + 1000;
            led_is_on = 1;
        }
        last_slave_cnt = new_cnt;
        first_poll_done = 1;
    }
    master_state = MSTATE_IDLE;
}

void OnMasterError(Modbus_Error_t error) {
    (void)error;
    master_state = MSTATE_IDLE; /* Fehler ignorieren, beim nächsten Polling neu versuchen */
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

  /* Erzwungener PULLDOWN für PC13, da das Board offensichtlich Active-HIGH nutzt! */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTN_intern_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_intern_GPIO_Port, &GPIO_InitStruct);

  /* Boot-Blinken */
  for (int i=0; i<6; i++) {
      HAL_GPIO_TogglePin(LED_intern_GPIO_Port, LED_intern_Pin);
      HAL_Delay(100);
  }
  // LED INITIAL AUS (Active-HIGH -> RESET = AUS)
  HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);

  uint8_t pending_master_cmd = 0;

  while (1)
  {
      // Active-HIGH: Wenn Pin auf 3.3V (HIGH / SET) gezogen wird = Gedrückt!
      GPIO_PinState current = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);

      if (current == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 500)) {
          last_btn_tick = HAL_GetTick(); // Debounce Timeout reset

          /* Eigene LED für 3s an (Active-HIGH -> SET = AN) */
          HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET); 
          led_off_tick = HAL_GetTick() + 3000;
          led_is_on = 1;

          /* Merken, dass wir unbedingt ein FC06 senden müssen! */
          pending_master_cmd = 1;
      }

      /* 2. PRIORITÄTS-MANAGEMENT FÜR DEN BUS */
      if (master_state == MSTATE_IDLE) {
          if (pending_master_cmd) {
              /* Prio 1: Unser eigener Tastendruck muss verschickt werden! */
              Modbus_Master_Request(&hmodbus, 1, MB_FC_WRITE_SINGLE_REGISTER, REG_MASTER_CMD, 3000);
              master_state = MSTATE_SENDING_CMD;
              pending_master_cmd = 0;
              last_poll_tick = HAL_GetTick(); // Timer zurücksetzen, um nicht sofort danach zu pollen
          }
          else if (HAL_GetTick() - last_poll_tick >= 100) {
              /* Prio 2: Polling, ob der Slave-Button gedrückt wurde */
              Modbus_Master_Request(&hmodbus, 1, MB_FC_READ_HOLDING_REGISTERS, REG_SLAVE_BTN_CNT, 1);
              master_state = MSTATE_POLLING_SLAVE;
              last_poll_tick = HAL_GetTick();
          }
      }

      /* 3. MODBUS BEDIENEN */
      if (master_state != MSTATE_IDLE) {
          Modbus_Master_Process(&hmodbus);
      }

      /* 4. LED TIMER PRÜFEN */
      if (led_is_on && HAL_GetTick() >= led_off_tick) {
          HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET); // LED AUS (RESET = AUS)
          led_is_on = 0;
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
