/**
  ******************************************************************************
  * @file           : mx_cortex_nvic.c
  * @brief          : CORTEX_NVIC Peripheral initialization
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the mx_stm32c5xx_hal_drivers_license.md file
  * in the same directory as the generated code.
  * If no mx_stm32c5xx_hal_drivers_license.md file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx_adc1.h"
#include "mx_cortex_nvic.h"
#include "mx_spi2.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype------------------------------------------------*/
/* Exported variables by reference--------------------------------------------*/

/******************************************************************************/
/* Exported functions for CORTEX_NVIC in HAL layer */
/******************************************************************************/
system_status_t mx_cortex_nvic_init(void)
{
  /* Configure the Priority grouping */
  HAL_CORTEX_NVIC_SetPriorityGrouping(HAL_CORTEX_NVIC_PRIORITY_GROUP_4);

  /* USART1: interrupt line always enabled. Harmless when USART1 is only used
     in polling mode (HAL_UART_Transmit/Receive do not enable any USART-level
     interrupt source, so the NVIC line never fires in that case); required
     when USART1 is used in interrupt mode (HAL_UART_Transmit_IT / _ReceiveToIdle_IT). */
  HAL_CORTEX_NVIC_SetPriority(USART1_IRQn, HAL_CORTEX_NVIC_PREEMP_PRIORITY_5, HAL_CORTEX_NVIC_SUB_PRIORITY_0);
  HAL_CORTEX_NVIC_EnableIRQ(USART1_IRQn);

  /* SPI2: interrupt line always enabled, harmless when unused (no interrupt
     source enabled at the peripheral level unless a _IT transfer is armed). */
  HAL_CORTEX_NVIC_SetPriority(SPI2_IRQn, HAL_CORTEX_NVIC_PREEMP_PRIORITY_5, HAL_CORTEX_NVIC_SUB_PRIORITY_0);
  HAL_CORTEX_NVIC_EnableIRQ(SPI2_IRQn);

  /* ADC1: interrupt line always enabled, harmless when unused. Used for the
     single-channel conversions triggered on demand by SPI2 START commands
     (HAL_ADC_REG_StartConv_IT). */
  HAL_CORTEX_NVIC_SetPriority(ADC1_IRQn, HAL_CORTEX_NVIC_PREEMP_PRIORITY_5, HAL_CORTEX_NVIC_SUB_PRIORITY_0);
  HAL_CORTEX_NVIC_EnableIRQ(ADC1_IRQn);

  return SYSTEM_OK;
}

/* NOTE: USART1_IRQHandler() is implemented in main.c, not here: it needs to
   re-arm HAL_UART_ReceiveToIdle_IT() right after HAL_UART_IRQHandler()
   returns (once rx_state is back to IDLE), which requires access to the
   application's RX buffer and re-arm flag. See main.c for details.

   SPI2 and ADC1 do not have this hazard: SPI_CloseTransfer()
   (stm32c5xx_hal_spi.c) and the ADC1 regular-group unitary-conversion
   completion path (stm32c5xx_hal_adc.c) both reset their group/global state
   to IDLE *before* calling the respective completion callback, so re-arming
   the next transfer/conversion synchronously from inside those callbacks (or,
   for ADC1, from a later SPI2 START command) is safe - their handlers can
   stay here, generic. */
void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(mx_spi2_gethandle());
}

void ADC1_IRQHandler(void)
{
  HAL_ADC_IRQHandler(mx_adc1_gethandle());
}
