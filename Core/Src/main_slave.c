/**
  * @file    main_slave.c (SLAVE - T3100 VOTING NODE)
  * @brief   Modbus-Slave mit Wunsch-Voting und Erlaubnis-LED-Feedback
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

/* --- Button State --- */
uint32_t last_btn_tick = 0;

void SystemClock_Config(void);

/* Wird aufgerufen wenn der Master ein Write (FC06/FC16) sendet */
void App_Slave_OnRegisterWrite(uint16_t reg_addr, uint16_t value) {
    // Der Master schreibt den erlaubten Status in REG_ERLAUBNIS!
    // register_map[reg_addr] wurde bereits durch die Modbus-Lib aktualisiert.
    if (reg_addr == REG_SAFETY_STOP && value == 1) {
        // Notaus! Alles verweigern.
        register_map[REG_ERLAUBNIS] = MODE_STANDBY;
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
  for(int i=0; i<3; i++) {
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
  }

  /* Starte im Standby Mode */
  register_map[REG_WUNSCH] = MODE_STANDBY;
  register_map[REG_ERLAUBNIS] = MODE_STANDBY;

  uint32_t led_blink_tick = 0;
  uint8_t led_state = 0;

  while (1)
  {
    /* 1. Modbus State Machine aufrechterhalten */
    Modbus_Slave_Listen(&hmodbus, register_map, APP_REGISTER_MAP_SIZE);

    /* 2. KNOPF PRÜFEN (Intern PC13 -> Active HIGH) zum Durchschalten des eigenen Wunsches */
    GPIO_PinState current = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);
    if (current == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 300)) {
        last_btn_tick = HAL_GetTick();

        // Zyklisch durchschalten: Standby (0) -> Charge (1) -> Discharge (2) -> Standby (0)
        uint16_t neuer_wunsch = register_map[REG_WUNSCH] + 1;
        if (neuer_wunsch > 2) neuer_wunsch = 0;
        
        register_map[REG_WUNSCH] = neuer_wunsch;
    }

    /* 3. LED ANSTEUERUNG BASIEREND AUF VOM MASTER ***ERLAUBTEN*** STATUS! */
    uint16_t erlaubnis = register_map[REG_ERLAUBNIS];
    
    if (erlaubnis == MODE_STANDBY) {
        // Standby = Aus
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
    } 
    else if (erlaubnis == MODE_CHARGE) {
        // Charge = Dauerhaft An
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
    } 
    else if (erlaubnis == MODE_DISCHARGE) {
        // Discharge = Blinken (jede 200ms)
        if (HAL_GetTick() - led_blink_tick > 200) {
            led_blink_tick = HAL_GetTick();
            led_state = !led_state;
            HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
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
