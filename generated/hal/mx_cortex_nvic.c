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
#include "mx_cortex_nvic.h"
#include "mx_usart1.h"

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

  return SYSTEM_OK;
}

/******************************************************************************/
/*                         Peripheral Interrupt Handlers                      */
/******************************************************************************/
/**
  * @brief  This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(mx_usart1_uart_gethandle());
}
