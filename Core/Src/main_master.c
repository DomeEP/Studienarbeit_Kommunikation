/**
 * @file main_master.c
 * @brief Master firmware entry point (Arbiter)
 */
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"

#include "system_arbiter.h"
#include "app_config.h"

void SystemClock_Config(void);

uint32_t last_btn_tick = 0;
uint32_t led_blink_tick = 0;
uint8_t  led_state = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USB_Device_Init();
  MX_TIM6_Init();

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTN_intern_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_intern_GPIO_Port, &GPIO_InitStruct);

  /* Boot blink sequence */
  for(int i=0; i<3; i++) {
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
  }

  Arbiter_Init(&huart2, &htim6);
  Arbiter_SetGlobalRunState(false);

  uint8_t master_wakeup_blinks = 0;
  uint32_t wakeup_tick = 0;

  while (1)
  {
      Arbiter_Process();

      /* Button: toggle system on/off */
      GPIO_PinState btn_state = HAL_GPIO_ReadPin(BTN_intern_GPIO_Port, BTN_intern_Pin);
      if (btn_state == GPIO_PIN_SET && (HAL_GetTick() - last_btn_tick > 500)) {
          last_btn_tick = HAL_GetTick();

          bool current_run_state = Arbiter_GetGlobalRunState();

          if (!current_run_state) {
              Arbiter_TriggerWakeup();
              master_wakeup_blinks = 6;
              wakeup_tick = HAL_GetTick();
              HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
          }
          Arbiter_SetGlobalRunState(!current_run_state);
      }

      /* Diagnostic LED */
      if (master_wakeup_blinks > 0) {
          if (HAL_GetTick() - wakeup_tick > 100) {
              wakeup_tick = HAL_GetTick();
              master_wakeup_blinks--;
              HAL_GPIO_TogglePin(LED_intern_GPIO_Port, LED_intern_Pin);
          }
      } else {
          if (!Arbiter_GetGlobalRunState()) {
              HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
          } else {
              uint16_t status = Arbiter_GetSystemMode();
              if (status == SYSTEM_MODE_CHARGE) {
                  HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_SET);
              } else if (status == SYSTEM_MODE_DISCHARGE) {
                  if (HAL_GetTick() - led_blink_tick > 100) {
                      led_blink_tick = HAL_GetTick();
                      led_state = !led_state;
                      HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                  }
              } else {
                  HAL_GPIO_WritePin(LED_intern_GPIO_Port, LED_intern_Pin, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) { (void)f; (void)l; }
#endif
