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
#define APP_MODE_LOOPBACK_TEST  2  /* USART1 self-test: requires PA9 (TX) jumpered to PA10 (RX) on this board */

#define APP_MODE                APP_MODE_COMMS_TEST

#if (APP_MODE != APP_MODE_ADC_DEBUG) && (APP_MODE != APP_MODE_COMMS_TEST) && (APP_MODE != APP_MODE_LOOPBACK_TEST)
#error "APP_MODE must be set to APP_MODE_ADC_DEBUG, APP_MODE_COMMS_TEST or APP_MODE_LOOPBACK_TEST"
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

#elif (APP_MODE == APP_MODE_COMMS_TEST)
/* =============================================================================
 * APP_MODE_COMMS_TEST: interrupt-driven PING/PONG link to the remote board
 * (remote: USART3 on PB3/PB4, sends "PING <n>\r\n" once a second).
 * USART1 carries only the protocol traffic - no extra debug text is sent on
 * it, so the link to the remote board stays clean; state below is meant to be
 * inspected via debugger/live-watch instead.
 * ===========================================================================*/
/* Set to 1 for a raw RX sanity check: PB15 toggles on every single byte
   received on USART1, bypassing PING/PONG parsing entirely. Confirms the
   RXNE interrupt/ISR chain fires at all when the remote board sends
   something, independent of IDLE-line detection and message framing.
   Set back to 0 for the normal PING/PONG protocol behavior. */
#define COMMS_RAW_RX_DEBUG      0

#define COMMS_RX_BUF_SIZE       32U
#define COMMS_TX_BUF_SIZE       32U

static uint8_t comms_rx_buf[COMMS_RX_BUF_SIZE];

static volatile uint32_t comms_error_count       = 0U;
static volatile uint32_t comms_last_error_codes   = 0U; /* HAL_UART_RECEIVE_ERROR_xxx bitmask, see stm32c5xx_hal_uart.h */
static volatile uint8_t  comms_rx_rearm_pending   = 0U;

/* PB15 toggles on every message received (raw mode: every single byte;
   normal mode: every complete "IDLE-terminated" message, regardless of
   whether it parses as a valid PING) - put a probe/LED on it for a visual,
   debugger-free check that reception is actually happening on this path. */
static void comms_pb15_init(void)
{
  HAL_RCC_GPIOB_EnableClock();

  hal_gpio_config_t gpio_config;
  gpio_config.mode        = HAL_GPIO_MODE_OUTPUT;
  gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.pull        = HAL_GPIO_PULL_NO;
  gpio_config.speed       = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.init_state  = HAL_GPIO_PIN_RESET;
  HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_15, &gpio_config);
}

#if (COMMS_RAW_RX_DEBUG == 1)
static volatile uint32_t comms_raw_rx_byte_count = 0U;
#else /* COMMS_RAW_RX_DEBUG == 0 */
static uint8_t comms_tx_buf[COMMS_TX_BUF_SIZE];

static volatile uint32_t comms_ping_rx_count = 0U;
static volatile uint32_t comms_pong_tx_count = 0U;
static volatile uint32_t comms_last_ping_n   = 0U;

/* Bumped when something IS received (no line error) but does not match the
   expected "PING <n>" text: a byte-for-byte copy is kept here so a debugger
   can inspect exactly what the remote board actually sent, to catch a format
   mismatch (wrong case, different separator, extra/missing characters, ...)
   that would otherwise fail silently (comms_ping_rx_count staying at 0 gives
   no clue whether nothing arrived at all, or something arrived unrecognized). */
static volatile uint32_t comms_unrecognized_count    = 0U;
static char              comms_last_unrecognized_msg[COMMS_RX_BUF_SIZE + 1U];
static volatile uint32_t comms_last_unrecognized_len = 0U;
#endif /* COMMS_RAW_RX_DEBUG */

static void comms_rx_arm(hal_uart_handle_t *huart)
{
#if (COMMS_RAW_RX_DEBUG == 1)
  (void)HAL_UART_ReceiveToIdle_IT(huart, comms_rx_buf, 1U); /* one byte at a time */
#else
  (void)HAL_UART_ReceiveToIdle_IT(huart, comms_rx_buf, sizeof(comms_rx_buf));
#endif /* COMMS_RAW_RX_DEBUG */
}

#if (COMMS_RAW_RX_DEBUG == 0)
/* Parses "PING <n>\r\n" and, on the same link, replies "PONG <n>\r\n" */
static void comms_process_message(hal_uart_handle_t *huart, const uint8_t *data, uint32_t size_byte)
{
  char msg[COMMS_RX_BUF_SIZE + 1U];
  uint32_t copy_len = (size_byte < COMMS_RX_BUF_SIZE) ? size_byte : COMMS_RX_BUF_SIZE;

  HAL_GPIO_TogglePin(HAL_GPIOB, HAL_GPIO_PIN_15);

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
  else
  {
    comms_unrecognized_count++;
    memcpy(comms_last_unrecognized_msg, msg, copy_len + 1U);
    comms_last_unrecognized_len = copy_len;
  }
}
#endif /* COMMS_RAW_RX_DEBUG == 0 */

/* IMPORTANT: do NOT call comms_rx_arm()/HAL_UART_ReceiveToIdle_IT() from
   here. HAL_UART_IRQHandler()'s non-DMA "reception to IDLE" completion path
   calls this callback and only AFTER it returns does it unconditionally reset
   huart->rx_state to HAL_UART_RX_STATE_IDLE - clobbering the RX_STATE_ACTIVE
   that a re-arm done in here would have just set. UART_RxISR_8BIT() then
   silently flushes every subsequent received byte because it gates on
   rx_state == ACTIVE, so the link goes deaf after exactly one message.
   Just flag that a re-arm is needed; the actual re-arm happens in
   USART1_IRQHandler() (see below), right after HAL_UART_IRQHandler() (and
   therefore this callback) has returned and rx_state is IDLE again - still
   inside the same interrupt, so there is no gap where reception is unarmed. */
void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte, hal_uart_rx_event_types_t rx_event)
{
  (void)rx_event;
#if (COMMS_RAW_RX_DEBUG == 1)
  (void)huart;
  (void)size_byte;
  comms_raw_rx_byte_count++;
  HAL_GPIO_TogglePin(HAL_GPIOB, HAL_GPIO_PIN_15);
#else
  comms_process_message(huart, comms_rx_buf, size_byte);
#endif /* COMMS_RAW_RX_DEBUG */
  comms_rx_rearm_pending = 1U;
}

/* PB15 also toggles here so a line error (framing/noise/overrun) on the real
   link to the remote board becomes visible even without a debugger - it just
   can't be told apart on the scope from a successful reception by itself;
   comms_last_error_codes (via debugger) is what distinguishes the two for
   real, and which exact error it is (needs USE_HAL_UART_GET_LAST_ERRORS=1
   in stm32c5xx_hal_conf.h, see HAL_UART_RECEIVE_ERROR_xxx in
   stm32c5xx_hal_uart.h: bit0=PE parity, bit1=NE noise, bit2=FE framing,
   bit3=ORE overrun, bit5=RTO timeout). */
void HAL_UART_ErrorCallback(hal_uart_handle_t *huart)
{
  comms_error_count++;
  comms_last_error_codes = HAL_UART_GetLastErrorCodes(huart);
  HAL_GPIO_TogglePin(HAL_GPIOB, HAL_GPIO_PIN_15);
  comms_rx_rearm_pending = 1U;
}

/* USART1 global interrupt handler: services the peripheral, then re-arms
   reception if HAL_UART_RxCpltCallback()/HAL_UART_ErrorCallback() flagged it
   as needed (see their comments for why this can't be done in-line). */
void USART1_IRQHandler(void)
{
  hal_uart_handle_t *huart = mx_usart1_uart_gethandle();

  HAL_UART_IRQHandler(huart);

  if (comms_rx_rearm_pending != 0U)
  {
    comms_rx_rearm_pending = 0U;
    comms_rx_arm(huart);
  }
}

#else /* APP_MODE_LOOPBACK_TEST */
/* =============================================================================
 * APP_MODE_LOOPBACK_TEST: local self-test, independent of the remote board.
 * Requires PA9 (TX) physically jumpered to PA10 (RX) on THIS board. This
 * firmware generates its own "PING <n>\r\n" once a second on USART1 and
 * expects to receive it right back through the wire; every time a message is
 * received, PB15 is toggled - put a probe/LED on it to check the whole
 * TX + wire + RX + interrupt chain on this board alone, with no dependency
 * on the link to the remote board.
 * Set APP_MODE back to APP_MODE_COMMS_TEST once this checks out, and remove
 * the PA9-PA10 jumper before reconnecting to the remote board.
 * ===========================================================================*/
#define LOOPBACK_TX_PERIOD_MS   1000U
#define LOOPBACK_TX_BUF_SIZE    32U
#define LOOPBACK_RX_BUF_SIZE    32U

static uint8_t loopback_tx_buf[LOOPBACK_TX_BUF_SIZE];
static uint8_t loopback_rx_buf[LOOPBACK_RX_BUF_SIZE];

static volatile uint32_t loopback_rx_count        = 0U;
static volatile uint32_t loopback_error_count     = 0U;
static volatile uint8_t  loopback_rx_rearm_pending = 0U;

static void loopback_rx_arm(hal_uart_handle_t *huart)
{
  (void)HAL_UART_ReceiveToIdle_IT(huart, loopback_rx_buf, sizeof(loopback_rx_buf));
}

static void loopback_pb15_init(void)
{
  HAL_RCC_GPIOB_EnableClock();

  hal_gpio_config_t gpio_config;
  gpio_config.mode        = HAL_GPIO_MODE_OUTPUT;
  gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.pull        = HAL_GPIO_PULL_NO;
  gpio_config.speed       = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.init_state  = HAL_GPIO_PIN_RESET;
  HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_15, &gpio_config);
}

/* Same "do not re-arm from here" hazard as APP_MODE_COMMS_TEST above: the
   actual re-arm happens in USART1_IRQHandler() below, right after
   HAL_UART_IRQHandler() returns and rx_state is IDLE again. */
void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte, hal_uart_rx_event_types_t rx_event)
{
  (void)huart;
  (void)size_byte;
  (void)rx_event;
  loopback_rx_count++;
  HAL_GPIO_TogglePin(HAL_GPIOB, HAL_GPIO_PIN_15);
  loopback_rx_rearm_pending = 1U;
}

void HAL_UART_ErrorCallback(hal_uart_handle_t *huart)
{
  (void)huart;
  loopback_error_count++;
  loopback_rx_rearm_pending = 1U;
}

void USART1_IRQHandler(void)
{
  hal_uart_handle_t *huart = mx_usart1_uart_gethandle();

  HAL_UART_IRQHandler(huart);

  if (loopback_rx_rearm_pending != 0U)
  {
    loopback_rx_rearm_pending = 0U;
    loopback_rx_arm(huart);
  }
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

#elif (APP_MODE == APP_MODE_COMMS_TEST)

  comms_pb15_init();
  comms_rx_arm(huart);

  while (1)
  {
    /* The PING/PONG exchange (including re-arming reception) runs entirely
       under the USART1 interrupt (see USART1_IRQHandler() above); the
       application is free to do other work here in the meantime. */
  }

#else /* APP_MODE_LOOPBACK_TEST */

  loopback_pb15_init();
  loopback_rx_arm(huart);

  uint32_t ping_n = 0U;

  while (1)
  {
    /* Re-arming reception runs under the USART1 interrupt, see
       USART1_IRQHandler() above. */
    int len = snprintf((char *)loopback_tx_buf, sizeof(loopback_tx_buf), "PING %lu\r\n", (unsigned long)ping_n);
    (void)HAL_UART_Transmit_IT(huart, loopback_tx_buf, (uint32_t)len);
    ping_n++;

    HAL_Delay(LOOPBACK_TX_PERIOD_MS);
  }

#endif /* APP_MODE */

} /* end main */
