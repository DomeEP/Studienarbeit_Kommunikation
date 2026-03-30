/* USER CODE BEGIN Header */
/**
  * @file    main.c (SLAVE - BIDIREKTIONALER MODBUS)
  * @brief   Empfängt Modbus-Befehle und speichert Button-Clicks im Register
  */
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "modbus_rtu.h"
#include "app_config.h"

Modbus_Handle_t hmodbus;
uint16_t register_map[APP_REGISTER_MAP_SIZE] = {0};

/* --- Timers & State --- */
uint32_t led_off_tick = 0;
uint8_t led_is_on = 0;

uint32_t last_btn_tick = 0;
GPIO_PinState last_btn_state = GPIO_PIN_SET;
uint16_t slave_btn_cnt = 0;

void SystemClock_Config(void);

/* Wird aufgerufen wenn der Master ein Write (FC06) sendet */
void App_Slave_OnRegisterWrite(uint16_t reg_addr, uint16_t value) {
    if (reg_addr == REG_MASTER_CMD) {
        /* Master hat uns geschickt, wie lange die LED an sein soll (z.B. 3000ms) */
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET); // LED AN (Active-HIGH)
        led_off_tick = HAL_GetTick() + value;
        led_is_on = 1;
        
        /* Register wieder leeren (verhindert erneutes auslesen von alten Werten) */
        register_map[REG_MASTER_CMD] = 0;
    }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* UART Interrupts aktivieren */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* Modbus Slave initialisieren (ID=1) */
  Modbus_Init(&hmodbus, &huart2, &htim6, APP_SLAVE_ID);
  hmodbus.write_reg_cb = App_Slave_OnRegisterWrite;

  /* Erzwungene PULLDOWN Aktivierung für PC13 */
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
  // LED AUS (Active-HIGH -> RESET = AUS)
  HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);

  while (1)
  {
    /* 1. MODBUS LAUSCHEN */
    Modbus_Slave_Listen(&hmodbus, register_map, APP_REGISTER_MAP_SIZE);

    /* 2. EIGENEN BUTTON PRÜFEN (Intern PC13) -> Active-HIGH */
    GPIO_PinState current = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);

    /* Wenn Pin auf 3.3V gezogen wird (Active-HIGH) */
    if (current == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 500)) {
        last_btn_tick = HAL_GetTick();

        /* Eigene LED für 1s an (Active-HIGH) */
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET); // LED AN
        led_off_tick = HAL_GetTick() + 1000;
        led_is_on = 1;

        /* Zähler hochsetzen und in Register packen -> der Master liest das per Polling! */
        slave_btn_cnt++;
        register_map[REG_SLAVE_BTN_CNT] = slave_btn_cnt;
    }

    /* 3. LED TIMER PRÜFEN */
    if (led_is_on && HAL_GetTick() >= led_off_tick) {
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET); // LED AUS (Active-HIGH)
        led_is_on = 0;
    }
  }
}

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
