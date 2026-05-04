/**
 * @file main_slave.c
 * @brief Slave firmware entry point (node)
 *
 * Demonstrates SlaveNode driver integration.
 * Button (PC13) cycles through desired modes,
 * LED reflects the master-granted operating state.
 */
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "modbus_rtu.h"
#include "app_config.h"
#include "slave_node.h"

uint32_t last_btn_tick = 0;

void SystemClock_Config(void);

void OnSystemEmergency(void) {
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

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTN_intern_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_intern_GPIO_Port, &GPIO_InitStruct);

  /* Boot blink */
  for(int i=0; i<3; i++) {
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
  }

  SlaveNode_Init(&huart2, &htim6, APP_SLAVE_ID, OnSystemEmergency);
  SlaveNode_SetNodeType(NODE_TYPE_INVERTER);

  uint32_t led_blink_tick = 0;
  uint8_t led_state = 0;
  uint16_t mein_wunsch = SYSTEM_MODE_STANDBY;

  uint8_t slave_wakeup_blinks = 0;
  uint32_t wakeup_tick = 0;

  while (1)
  {
    SlaveNode_Process();

    /* Button: cycle through Standby -> Charge -> Discharge */
    GPIO_PinState current = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);
    if (current == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 300)) {
        last_btn_tick = HAL_GetTick();

        mein_wunsch++;
        if (mein_wunsch >= SYSTEM_MODE_WAKEUP) mein_wunsch = SYSTEM_MODE_STANDBY;
        SlaveNode_SetDesiredMode(mein_wunsch);
    }

    /* React to master-granted mode */
    uint16_t erlaubnis = SlaveNode_ConsumeAllowedMode();

    if (erlaubnis == SYSTEM_MODE_WAKEUP) {
        slave_wakeup_blinks = 6;
        wakeup_tick = HAL_GetTick();
        HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
    }

    if (slave_wakeup_blinks > 0) {
        if (HAL_GetTick() - wakeup_tick > 100) {
            wakeup_tick = HAL_GetTick();
            slave_wakeup_blinks--;
            HAL_GPIO_TogglePin(LED_intern_GPIO_Port, LED_intern_Pin);
        }
    } else {
        uint16_t aktueller_mode = SlaveNode_GetAllowedMode();

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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) { (void)f; (void)l; }
#endif
