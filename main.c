/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* =============================================================================
 * USART1 (PA9=TX / PA10=RX) is shared between two mutually exclusive test
 * setups. Pick which one gets compiled in by setting APP_MODE below, then
 * rebuild - the two cannot run at the same time on the same physical link.
 * ===========================================================================*/
#define APP_MODE_ADC_DEBUG      0  /* USART1 streams the 8-channel 4-20mA ADC report (polling, human-readable) */
#define APP_MODE_COMMS_TEST     1  /* USART1 is the interrupt-driven PING/PONG link to the remote board */

#define APP_MODE                APP_MODE_COMMS_TEST

#if (APP_MODE != APP_MODE_ADC_DEBUG) && (APP_MODE != APP_MODE_COMMS_TEST)
#error "APP_MODE must be set to APP_MODE_ADC_DEBUG or APP_MODE_COMMS_TEST"
#endif

#define UART_TX_TIMEOUT_MS      100U

#if (APP_MODE == APP_MODE_ADC_DEBUG)
/* =============================================================================
 * APP_MODE_ADC_DEBUG: 8-channel 4-20mA ADC report over USART1 (polling)
 * ===========================================================================*/
#define ADC_VREF_MV             3300U  /* External VREF+ = 3.3V */
#define ADC_NB_CHANNELS         8U
#define ADC_CONV_TIMEOUT_MS     10U
#define LOOP_PERIOD_MS          500U

/* 4-20mA current loop conditioning: 4mA -> 0.6V, 20mA -> 3.0V at the ADC input */
#define LOOP_VOLTAGE_MIN_MV     600
#define LOOP_VOLTAGE_MAX_MV     3000
#define LOOP_CURRENT_MIN_UA     4000
#define LOOP_CURRENT_MAX_UA     20000

/* PA0..PA7 -> ADC1_IN0..ADC1_IN7 */
static const hal_adc_channel_t adc_channels[ADC_NB_CHANNELS] =
{
  HAL_ADC_CHANNEL_0, HAL_ADC_CHANNEL_1, HAL_ADC_CHANNEL_2, HAL_ADC_CHANNEL_3,
  HAL_ADC_CHANNEL_4, HAL_ADC_CHANNEL_5, HAL_ADC_CHANNEL_6, HAL_ADC_CHANNEL_7
};

static void uart_print(hal_uart_handle_t *huart, const char *str)
{
  HAL_UART_Transmit(huart, (const uint8_t *)str, (uint32_t)strlen(str), UART_TX_TIMEOUT_MS);
}

#else /* APP_MODE_COMMS_TEST */
/* =============================================================================
 * APP_MODE_COMMS_TEST: interrupt-driven PING/PONG link to the remote board
 * (remote: USART3 on PB3/PB4, sends "PING <n>\r\n" once a second).
 * USART1 carries only the protocol traffic - no extra debug text is sent on
 * it, so the link to the remote board stays clean; state below is meant to be
 * inspected via debugger/live-watch instead.
 * ===========================================================================*/
#define COMMS_RX_BUF_SIZE       32U
#define COMMS_TX_BUF_SIZE       32U

static uint8_t comms_rx_buf[COMMS_RX_BUF_SIZE];
static uint8_t comms_tx_buf[COMMS_TX_BUF_SIZE];

static volatile uint32_t comms_ping_rx_count = 0U;
static volatile uint32_t comms_pong_tx_count = 0U;
static volatile uint32_t comms_error_count   = 0U;
static volatile uint32_t comms_last_ping_n   = 0U;

static void comms_rx_arm(hal_uart_handle_t *huart)
{
  (void)HAL_UART_ReceiveToIdle_IT(huart, comms_rx_buf, sizeof(comms_rx_buf));
}

/* Parses "PING <n>\r\n" and, on the same link, replies "PONG <n>\r\n" */
static void comms_process_message(hal_uart_handle_t *huart, const uint8_t *data, uint32_t size_byte)
{
  char msg[COMMS_RX_BUF_SIZE + 1U];
  uint32_t copy_len = (size_byte < COMMS_RX_BUF_SIZE) ? size_byte : COMMS_RX_BUF_SIZE;

  memcpy(msg, data, copy_len);
  msg[copy_len] = '\0';

  unsigned long n;
  if (sscanf(msg, "PING %lu", &n) == 1)
  {
    comms_ping_rx_count++;
    comms_last_ping_n = n;

    /* Loop pings at 1Hz, far slower than the ~1ms it takes to shift this
       reply out at 115200 baud, so a previous Transmit_IT is always done
       by the time the next PING arrives: no queuing/busy-check needed. */
    int len = snprintf((char *)comms_tx_buf, sizeof(comms_tx_buf), "PONG %lu\r\n", n);

    if (HAL_UART_Transmit_IT(huart, comms_tx_buf, (uint32_t)len) == HAL_OK)
    {
      comms_pong_tx_count++;
    }
    else
    {
      comms_error_count++;
    }
  }
}

void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte, hal_uart_rx_event_types_t rx_event)
{
  (void)rx_event;
  comms_process_message(huart, comms_rx_buf, size_byte);
  comms_rx_arm(huart);
}

void HAL_UART_ErrorCallback(hal_uart_handle_t *huart)
{
  comms_error_count++;
  comms_rx_arm(huart);
}

#endif /* APP_MODE */

/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }

  /*
    * You can start your application code here
    */
  hal_uart_handle_t *huart = mx_usart1_uart_gethandle();

#if (APP_MODE == APP_MODE_ADC_DEBUG)

  hal_adc_handle_t *hadc = mx_adc1_gethandle();
  char line[192];

  if ((HAL_ADC_Start(hadc) != HAL_OK) || (HAL_ADC_Calibrate(hadc) != HAL_OK))
  {
    uart_print(huart, "ADC1 activation/calibration failed\r\n");
    while (1) {}
  }

  while (1)
  {
    int len = snprintf(line, sizeof(line), "4-20mA readings (Vref=3.3V):");

    for (uint32_t idx = 0U; idx < ADC_NB_CHANNELS; idx++)
    {
      hal_adc_channel_config_t channel_config;
      channel_config.group          = HAL_ADC_GROUP_REGULAR;
      channel_config.sequencer_rank = 1U;
      channel_config.sampling_time  = HAL_ADC_SAMPLING_TIME_48CYCLES;
      channel_config.input_mode     = HAL_ADC_IN_SINGLE_ENDED;

      HAL_ADC_SetConfigChannel(hadc, adc_channels[idx], &channel_config);

      HAL_ADC_REG_StartConv(hadc);
      HAL_ADC_REG_PollForConv(hadc, ADC_CONV_TIMEOUT_MS);
      int32_t raw_value = HAL_ADC_REG_ReadConversionData(hadc);
      HAL_ADC_REG_StopConv(hadc);

      int32_t voltage_mv = HAL_ADC_CALC_DATA_TO_VOLTAGE(ADC_VREF_MV, raw_value, HAL_ADC_RESOLUTION_12_BIT);

      /* Linear scaling from conditioned voltage to loop current (integer math: nano.specs printf has no float support) */
      int32_t current_ua = LOOP_CURRENT_MIN_UA + (voltage_mv - LOOP_VOLTAGE_MIN_MV) *
                            (LOOP_CURRENT_MAX_UA - LOOP_CURRENT_MIN_UA) / (LOOP_VOLTAGE_MAX_MV - LOOP_VOLTAGE_MIN_MV);
      int32_t current_ua_abs = (current_ua < 0) ? -current_ua : current_ua;
      uint32_t current_ma_int  = (uint32_t)(current_ua_abs / 1000);
      uint32_t current_ma_frac = (uint32_t)((current_ua_abs / 10) % 100);

      len += snprintf(&line[len], sizeof(line) - (size_t)len, " PA%lu=%4ld(%s%lu.%02lumA)",
                       (unsigned long)idx, (long)raw_value, (current_ua < 0) ? "-" : "",
                       (unsigned long)current_ma_int, (unsigned long)current_ma_frac);
    }

    len += snprintf(&line[len], sizeof(line) - (size_t)len, "\r\n");

    HAL_UART_Transmit(huart, (uint8_t *)line, (uint32_t)len, UART_TX_TIMEOUT_MS);

    HAL_Delay(LOOP_PERIOD_MS);
  }

#else /* APP_MODE_COMMS_TEST */

  comms_rx_arm(huart);

  while (1)
  {
    /* The PING/PONG exchange runs entirely under the USART1 interrupt
       (see HAL_UART_RxCpltCallback/ErrorCallback above); the application
       is free to do other work here in the meantime. */
  }

#endif /* APP_MODE */

} /* end main */
