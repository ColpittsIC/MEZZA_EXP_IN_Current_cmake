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
 * Integrated test firmware: all three peripherals run at the same time, fully
 * interrupt-driven, no polling/timeout anywhere in the main loop.
 *   - USART1 (PA9/PA10)   : PING/PONG link to the remote board.
 *   - SPI2   (PB12..PB15) : Slave side of a 4-byte command/response protocol
 *                           the remote board (Master) uses to read this
 *                           board's 8-channel 4-20mA ADC1 input.
 *   - ADC1   (PA0..PA7)   : driven on demand by the SPI2 protocol below (one
 *                           channel converted per SPI "START" command).
 * USART1 carries the real PING/PONG protocol with the remote board, so - same
 * rule as before - nothing else is ever printed on it: SPI2/ADC1 state is
 * kept in the debugger-inspectable variables below instead of being logged
 * over the wire, to avoid corrupting that protocol with extra text.
 * ===========================================================================*/
#define UART_TX_TIMEOUT_MS      100U

/* ============================== ADC1 (4-20mA) ============================== */
/* PA0..PA7 -> ADC1_IN0..ADC1_IN7, CH index 0..7 maps directly to this order
   (fixed, arbitrary choice - only mapping used by this firmware). External
   VREF+ = 3.3V, same as the Master board's own ADC reference. */
#define ADC_NB_CHANNELS         8U

static const hal_adc_channel_t adc_channels[ADC_NB_CHANNELS] =
{
  HAL_ADC_CHANNEL_0, HAL_ADC_CHANNEL_1, HAL_ADC_CHANNEL_2, HAL_ADC_CHANNEL_3,
  HAL_ADC_CHANNEL_4, HAL_ADC_CHANNEL_5, HAL_ADC_CHANNEL_6, HAL_ADC_CHANNEL_7
};

typedef enum
{
  ADC_CH_STATE_IDLE  = 0, /* no conversion requested yet for adc_active_channel */
  ADC_CH_STATE_BUSY  = 1, /* HAL_ADC_REG_StartConv_IT() called, EOC pending */
  ADC_CH_STATE_READY = 2  /* adc_last_raw[adc_active_channel] holds a fresh result */
} adc_ch_state_t;

/* Last known raw 12-bit code per channel - updated as channels get polled via
   SPI2, readable via debugger/live-watch. */
static volatile int32_t       adc_last_raw[ADC_NB_CHANNELS];
static volatile uint8_t       adc_active_channel = 0U; /* channel of the most recent SPI2 START */
static volatile adc_ch_state_t adc_conv_state     = ADC_CH_STATE_IDLE;
static volatile uint32_t      adc_error_count     = 0U;
static volatile uint32_t      adc_last_error_codes = 0U; /* HAL_ADC_ERROR_xxx bitmask, see stm32c5xx_hal_adc.h */

static void adc_start_channel(hal_adc_handle_t *hadc, uint8_t channel)
{
  hal_adc_channel_config_t channel_config;
  channel_config.group          = HAL_ADC_GROUP_REGULAR;
  channel_config.sequencer_rank = 1U;
  channel_config.sampling_time  = HAL_ADC_SAMPLING_TIME_48CYCLES;
  channel_config.input_mode     = HAL_ADC_IN_SINGLE_ENDED;

  HAL_ADC_SetConfigChannel(hadc, adc_channels[channel], &channel_config);

  adc_active_channel = channel;
  adc_conv_state      = ADC_CH_STATE_BUSY;

  (void)HAL_ADC_REG_StartConv_IT(hadc);
}

/* Fires once the single (sequencer_length == 1, see mx_adc1.c) regular
   conversion started by adc_start_channel() completes. Safe to read the
   result directly here: hadc->group_state[REGULAR] is reset to IDLE by the
   driver *before* this callback runs (stm32c5xx_hal_adc.c), same safe
   ordering as SPI2 - unlike USART1's IDLE-reception path. */
void HAL_ADC_REG_UnitaryConvCpltCallback(hal_adc_handle_t *hadc)
{
  adc_last_raw[adc_active_channel] = HAL_ADC_REG_ReadConversionData(hadc);
  adc_conv_state = ADC_CH_STATE_READY;
}

void HAL_ADC_ErrorCallback(hal_adc_handle_t *hadc)
{
  adc_error_count++;
  adc_last_error_codes = HAL_ADC_GetLastErrorCodes(hadc);
  adc_conv_state = ADC_CH_STATE_IDLE; /* let the next SPI2 START retry cleanly */
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

/* ======================= SPI2 (Slave: ADC read-out protocol) ================= */
/* SPI2 Slave side of a fixed 4-byte command/response protocol with the remote
   board (Master), which reads this board's 8 4-20mA ADC1 channels over SPI.
   Pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI - mode 0 (CPOL=0/CPHA=0),
   8-bit, MSB first, hardware NSS (one CS pulse per 4-byte transfer).
   Master TX: [CMD, CH, 0x00, 0x00]
   Slave  TX: [STATUS, RAW_HI, RAW_LO, CH_ECHO]
     CMD 0x01 START: (re)start a conversion on channel CH (0..7).
     CMD 0x02 POLL : ask whether the result for CH is ready.
     STATUS: 0x02 READY (RAW_HI/RAW_LO valid for channel CH_ECHO), anything
             else (0x00 idle, 0x01 busy) means "not ready yet" to the Master.
     RAW_HI/RAW_LO: raw 12-bit ADC code, big-endian, NOT converted to a
             voltage/current here - the Master does that itself.
     CH_ECHO: channel the STATUS/RAW data actually refers to, so the Master
             can discard a response that doesn't match what it just asked
             for (see the pipelining note below).
   NOTE on full-duplex pipelining (not a bug, see also the byte-echo test this
   replaced): the response transmitted during a transfer is the one prepared
   from the PREVIOUS transfer's command, never the one arriving in the same
   transfer - TX and RX shift on the same clock edges. The Master is expected
   to send repeated POLL frames back-to-back (no artificial delay) until it
   sees STATUS==READY with CH_ECHO matching what it asked for. */
#define SPI_FRAME_SIZE          4U
#define SPI_CMD_START           0x01U
#define SPI_CMD_POLL            0x02U
#define SPI_STATUS_IDLE         0x00U
#define SPI_STATUS_BUSY         0x01U
#define SPI_STATUS_READY        0x02U

static uint8_t spi_tx_buf[SPI_FRAME_SIZE] = { SPI_STATUS_IDLE, 0U, 0U, 0U };
static uint8_t spi_rx_buf[SPI_FRAME_SIZE];

static volatile uint32_t spi_xfer_count       = 0U;
static volatile uint32_t spi_error_count      = 0U;
static volatile uint32_t spi_last_error_codes = 0U; /* HAL_SPI_ERROR_xxx bitmask, see stm32c5xx_hal_spi.h */

static void spi_rearm(hal_spi_handle_t *hspi)
{
  (void)HAL_SPI_TransmitReceive_IT(hspi, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE);
}

/* Safe to re-arm directly from here, unlike the USART1 IDLE-reception path
   above: SPI_CloseTransfer() (stm32c5xx_hal_spi.c) resets hspi->global_state
   to IDLE *before* calling this callback, so there is no race and no need to
   defer the re-arm to the ISR/main loop. */
void HAL_SPI_TxRxCpltCallback(hal_spi_handle_t *hspi)
{
  uint8_t cmd = spi_rx_buf[0];
  uint8_t ch  = spi_rx_buf[1] & 0x07U; /* fold any out-of-range value into the valid 0..7 set */

  spi_xfer_count++;
  spi_rearm(hspi); /* keep the next transfer pre-armed before touching anything else */

  if (cmd == SPI_CMD_START)
  {
    adc_start_channel(mx_adc1_gethandle(), ch);
    spi_tx_buf[0] = SPI_STATUS_BUSY;
    spi_tx_buf[1] = 0U;
    spi_tx_buf[2] = 0U;
    spi_tx_buf[3] = ch;
  }
  else if (cmd == SPI_CMD_POLL)
  {
    if (ch != adc_active_channel)
    {
      /* Polling a channel that isn't the one currently tracked: report the
         tracked channel's status instead, with its own CH_ECHO, so the
         Master's mismatch check correctly discards this reply. */
      spi_tx_buf[0] = (adc_conv_state == ADC_CH_STATE_READY) ? SPI_STATUS_READY : SPI_STATUS_BUSY;
      spi_tx_buf[3] = adc_active_channel;
    }
    else if (adc_conv_state == ADC_CH_STATE_READY)
    {
      int32_t raw = adc_last_raw[adc_active_channel];
      spi_tx_buf[0] = SPI_STATUS_READY;
      spi_tx_buf[1] = (uint8_t)((raw >> 8) & 0xFFU);
      spi_tx_buf[2] = (uint8_t)(raw & 0xFFU);
      spi_tx_buf[3] = adc_active_channel;
    }
    else
    {
      spi_tx_buf[0] = (adc_conv_state == ADC_CH_STATE_BUSY) ? SPI_STATUS_BUSY : SPI_STATUS_IDLE;
      spi_tx_buf[1] = 0U;
      spi_tx_buf[2] = 0U;
      spi_tx_buf[3] = ch;
    }
  }
  else
  {
    /* Unknown command: report idle, no state change. */
    spi_tx_buf[0] = SPI_STATUS_IDLE;
    spi_tx_buf[1] = 0U;
    spi_tx_buf[2] = 0U;
    spi_tx_buf[3] = ch;
  }
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
    /* USART1 PING/PONG, SPI2 command/response, and the ADC1 conversions SPI2
       triggers on demand all run entirely under their own interrupts (see
       USART1_IRQHandler() / HAL_SPI_TxRxCpltCallback() /
       HAL_ADC_REG_UnitaryConvCpltCallback() above); the main loop has nothing
       left to do. */
  }
} /* end main */
