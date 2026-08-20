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
 * Integrated test firmware: all three peripherals run at the same time.
 *   - ADC1  (PA0..PA7)   : 8-channel 4-20mA loop reading, polled in the main loop.
 *   - USART1(PA9/PA10)   : interrupt-driven PING/PONG link to the remote board.
 *   - SPI2  (PB12..PB15) : interrupt-driven Slave echo (byte+1) with the remote
 *                          board's SPI Master.
 * USART1 carries the real PING/PONG protocol with the remote board, so - same
 * rule as before - nothing else is ever printed on it: ADC and SPI results are
 * kept in the debugger-inspectable state below instead of being logged over
 * the wire, to avoid corrupting that protocol with extra text.
 * ===========================================================================*/
#define UART_TX_TIMEOUT_MS      100U

/* ============================== ADC1 (4-20mA) ============================== */
#define ADC_VREF_MV             3300U  /* External VREF+ = 3.3V */
#define ADC_NB_CHANNELS         8U
#define ADC_CONV_TIMEOUT_MS     10U
#define ADC_LOOP_PERIOD_MS      500U

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

typedef struct
{
  int32_t raw;
  int32_t current_ua;
} adc_channel_result_t;

/* Latest reading per channel - inspect via debugger/live-watch. */
static volatile adc_channel_result_t adc_results[ADC_NB_CHANNELS];

static void adc_read_all_channels(hal_adc_handle_t *hadc)
{
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

    adc_results[idx].raw        = raw_value;
    adc_results[idx].current_ua = current_ua;
  }
}

/* ============================ USART1 (PING/PONG) ============================ */
/* Interrupt-driven PING/PONG link to the remote board (remote: USART3 on
   PB3/PB4, sends "PING <n>\r\n" once a second). USART1 carries only the
   protocol traffic - no extra debug text is sent on it; state below is meant
   to be inspected via debugger/live-watch instead. */
#define COMMS_RX_BUF_SIZE       32U
#define COMMS_TX_BUF_SIZE       32U

static uint8_t comms_rx_buf[COMMS_RX_BUF_SIZE];
static uint8_t comms_tx_buf[COMMS_TX_BUF_SIZE];

static volatile uint32_t comms_error_count       = 0U;
static volatile uint32_t comms_last_error_codes  = 0U; /* HAL_UART_RECEIVE_ERROR_xxx bitmask, see stm32c5xx_hal_uart.h */
static volatile uint8_t  comms_rx_rearm_pending  = 0U;
static volatile uint32_t comms_ping_rx_count     = 0U;
static volatile uint32_t comms_pong_tx_count     = 0U;
static volatile uint32_t comms_last_ping_n       = 0U;

/* Bumped when something IS received (no line error) but does not match the
   expected "PING <n>" text: a byte-for-byte copy is kept here so a debugger
   can inspect exactly what the remote board actually sent, to catch a format
   mismatch (wrong case, different separator, extra/missing characters, ...)
   that would otherwise fail silently (comms_ping_rx_count staying at 0 gives
   no clue whether nothing arrived at all, or something arrived unrecognized). */
static volatile uint32_t comms_unrecognized_count    = 0U;
static char              comms_last_unrecognized_msg[COMMS_RX_BUF_SIZE + 1U];
static volatile uint32_t comms_last_unrecognized_len = 0U;

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
  else
  {
    comms_unrecognized_count++;
    memcpy(comms_last_unrecognized_msg, msg, copy_len + 1U);
    comms_last_unrecognized_len = copy_len;
  }
}

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
  comms_process_message(huart, comms_rx_buf, size_byte);
  comms_rx_rearm_pending = 1U;
}

/* comms_last_error_codes (via debugger) reports which exact error occurred
   (needs USE_HAL_UART_GET_LAST_ERRORS=1 in stm32c5xx_hal_conf.h, see
   HAL_UART_RECEIVE_ERROR_xxx in stm32c5xx_hal_uart.h: bit0=PE parity,
   bit1=NE noise, bit2=FE framing, bit3=ORE overrun, bit5=RTO timeout). */
void HAL_UART_ErrorCallback(hal_uart_handle_t *huart)
{
  comms_error_count++;
  comms_last_error_codes = HAL_UART_GetLastErrorCodes(huart);
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

/* =============================== SPI2 (echo) ================================ */
/* SPI2 Slave echo test with the remote board (SPI Master).
   Pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI - mode 0 (CPOL=0/CPHA=0),
   8-bit, MSB first, hardware NSS. The master periodically starts a 1-byte
   full-duplex transfer carrying an incrementing counter; this board replies
   with (received_byte + 1) mod 256, always keeping the next transfer
   pre-armed so it never misses a clock burst from the master.
   NOTE on full-duplex pipelining (not a bug): the byte THIS board transmits
   during a given transfer is the reply to the PREVIOUS transfer's received
   byte, not to the one arriving in that same transfer - TX and RX shift on
   the same clock edges, so a slave cannot react to and echo a byte within
   the same transfer it arrives in. The master will see its own byte "one
   transfer behind"; that is expected, not something to fix.
   Same rule as USART1: no debug text is sent anywhere for this test either
   (SPI2 doesn't share a wire with anything, but USART1 is busy with the real
   PING/PONG protocol) - state is kept here for debugger/live-watch instead. */
static uint8_t spi_tx_byte = 0U;
static uint8_t spi_rx_byte = 0U;

static volatile uint32_t spi_xfer_count       = 0U;
static volatile uint32_t spi_error_count      = 0U;
static volatile uint32_t spi_last_error_codes = 0U; /* HAL_SPI_ERROR_xxx bitmask, see stm32c5xx_hal_spi.h */
static volatile uint8_t  spi_last_rx          = 0U;
static volatile uint8_t  spi_last_tx          = 0U;

static void spi_rearm(hal_spi_handle_t *hspi)
{
  (void)HAL_SPI_TransmitReceive_IT(hspi, &spi_tx_byte, &spi_rx_byte, 1U);
}

/* Safe to re-arm directly from here, unlike the USART1 IDLE-reception path
   above: SPI_CloseTransfer() (stm32c5xx_hal_spi.c) resets hspi->global_state
   to IDLE *before* calling this callback, so there is no race and no need to
   defer the re-arm to the ISR/main loop. */
void HAL_SPI_TxRxCpltCallback(hal_spi_handle_t *hspi)
{
  spi_last_rx = spi_rx_byte;
  spi_tx_byte = (uint8_t)(spi_last_rx + 1U);
  spi_last_tx = spi_tx_byte;
  spi_xfer_count++;

  spi_rearm(hspi);
}

void HAL_SPI_ErrorCallback(hal_spi_handle_t *hspi)
{
  spi_error_count++;
  spi_last_error_codes = HAL_SPI_GetLastErrorCodes(hspi);
  spi_rearm(hspi);
}

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
  hal_adc_handle_t  *hadc  = mx_adc1_gethandle();
  hal_uart_handle_t *huart = mx_usart1_uart_gethandle();
  hal_spi_handle_t  *hspi  = mx_spi2_gethandle();

  if ((HAL_ADC_Start(hadc) != HAL_OK) || (HAL_ADC_Calibrate(hadc) != HAL_OK))
  {
    /* Fatal ADC bring-up failure: nothing else will ever run on USART1 after
       this, so a one-shot diagnostic line here does not risk corrupting the
       PING/PONG protocol. */
    static const char adc_fail_msg[] = "ADC1 activation/calibration failed\r\n";
    HAL_UART_Transmit(huart, (const uint8_t *)adc_fail_msg, (uint32_t)(sizeof(adc_fail_msg) - 1U), UART_TX_TIMEOUT_MS);
    while (1) {}
  }

  comms_rx_arm(huart);
  spi_rearm(hspi);

  while (1)
  {
    /* USART1 PING/PONG and SPI2 echo run entirely under their own interrupts
       (see USART1_IRQHandler() / HAL_SPI_TxRxCpltCallback() above); the main
       loop is only responsible for the polled ADC readings. */
    adc_read_all_channels(hadc);
    HAL_Delay(ADC_LOOP_PERIOD_MS);
  }
} /* end main */
