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
#include "slave_node.h"


/* --- Button State --- */
uint32_t last_btn_tick = 0;

void SystemClock_Config(void);

/* Optional: Eigener Callback, falls der Master NOTAUS drückt */
void OnSystemEmergency(void) {
    // LED ausmachen, Leistungselektronik trennen
    HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
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

  /* PULLDOWN für den Button auf PC13 */
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

  /* -------------------------------------------------------------------
   *   SLAVE NODE INITIALISIERUNG
   * ------------------------------------------------------------------- */
  // Initialisiert die gesamte Kommunikation als Knoten mit ID 1
  SlaveNode_Init(&huart2, &htim6, APP_SLAVE_ID, OnSystemEmergency);

  // Deklariere diese Platine als Inverter (Master des DC-Microgrids)
  SlaveNode_SetNodeType(NODE_TYPE_INVERTER);

  uint32_t led_blink_tick = 0;
  uint8_t led_state = 0;
  uint16_t mein_wunsch = SYSTEM_MODE_STANDBY;

  uint8_t slave_wakeup_blinks = 0;
  uint32_t wakeup_tick = 0;

  while (1)
  {
    /* 1. KNOTEN-KOMMUNIKATION AM LEBEN HALTEN */
    SlaveNode_Process();

    /* 2. SENSORIK AUSWERTEN & WUNSCH ÄUßERN */
    GPIO_PinState current = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);
    if (current == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 300)) {
        last_btn_tick = HAL_GetTick();

        // Zyklisch durchschalten: Standby -> Charge -> Discharge -> Standby
        mein_wunsch++;
        // WAKEUP Modus (3) überspringen, da dieser nur vom Master kommt
        if (mein_wunsch >= SYSTEM_MODE_WAKEUP) mein_wunsch = SYSTEM_MODE_STANDBY;
        
        // Dem Treiber unseren physikalischen Wunsch übergeben!
        SlaveNode_SetDesiredMode(mein_wunsch);
    }

    /* 3. HARDWARE BASIEREND AUF DER MASTER-ERLAUBNIS STEUERN */
    uint16_t erlaubnis = SlaveNode_ConsumeAllowedMode();

    if (erlaubnis == SYSTEM_MODE_WAKEUP) {
        // Start non-blocking blink sequence (3x toggeln = 6 Zustandswechsel)
        slave_wakeup_blinks = 6;
        wakeup_tick = HAL_GetTick();
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
    }
    
    // Wakeup Blink Sequenz Priorität
    if (slave_wakeup_blinks > 0) {
        if (HAL_GetTick() - wakeup_tick > 100) {
            wakeup_tick = HAL_GetTick();
            slave_wakeup_blinks--;
            HAL_GPIO_TogglePin(LED_intern_GPIO_Port, LED_intern_Pin);
        }
    } else {
        // Normale Auswertung
        uint16_t aktueller_mode = SlaveNode_GetAllowedMode(); // gibt nur noch echten Mode zurück
        
        if (aktueller_mode == SYSTEM_MODE_STANDBY) {
            HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
        } 
        else if (aktueller_mode == SYSTEM_MODE_CHARGE) {
            HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
        } 
        else if (aktueller_mode == SYSTEM_MODE_DISCHARGE) {
            if (HAL_GetTick() - led_blink_tick > 200) {
                led_blink_tick = HAL_GetTick();
                led_state = !led_state;
                HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
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
