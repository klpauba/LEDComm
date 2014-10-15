/*
 * Copyright (c) 2013, 2014 Kevin L. Pauba
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the LEDComm Driver for ChibiOS/RT.
 * 
 * Author: Kevin L. Pauba <klpauba@gmail.com>
 *
 */

/**
 * @file    ledcomm-uart.c
 * @brief   LEDComm UART Driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "ledcomm-uart.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#if LEDCOMM_USE_LUART1 || defined(__DOXYGEN__)
LEDCommUARTDriver_t LUART1;
#endif
#if LEDCOMM_USE_LUART2 || defined(__DOXYGEN__)
LEDCommUARTDriver_t LUART2;
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static inline void ledCommOn(LEDCommUARTDriver_t *ldp) {
    palSetPad(ldp->config->anode_port, ldp->config->anode_pad);
    palClearPad(ldp->config->cathode_port, ldp->config->cathode_pad);
}

static inline void ledCommOff(LEDCommUARTDriver_t *ldp) {
    palClearPad(ldp->config->anode_port, ldp->config->anode_pad);
    palClearPad(ldp->config->cathode_port, ldp->config->cathode_pad);
}

static inline void ledCommReverse(LEDCommUARTDriver_t *ldp) {
    palClearPad(ldp->config->anode_port, ldp->config->anode_pad);
    palSetPad(ldp->config->cathode_port, ldp->config->cathode_pad);
}

static inline void ledCommInitPad(LEDCommUARTDriver_t *ldp) {
    palSetPadMode((ldp)->config->anode_port, (ldp)->config->anode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palSetPadMode((ldp)->config->cathode_port, (ldp)->config->cathode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
}

static inline void linkUp(LEDCommUARTDriver_t *l) {
    l->link = 1;
#if CH_USE_EVENTS
#if LEDCOMM_THREADED
    chSysLock();
#else
    chSysLockFromIsr();
#endif
    chnAddFlagsI(l, CHN_CONNECTED);
#if LEDCOMM_THREADED
    chSysUnlock();
#else
    chSysUnlockFromIsr();
#endif
#endif /* CH_USE_EVENTS */
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   LEDComm UART Driver initialization.
 * @note    This function is <B>NOT</B> implicitly invoked by @p halInit(), you
 *          must explicitly initialize the driver.
 *
 * @init
 */
void luartInit(void) {
}

/**
 * @brief   Initializes the standard part of a @p LEDCommUARTDriver structure.
 *
 * @param[out] luartp    pointer to the @p LEDCommUARTDriver object
 *
 * @init
 */
void luartObjectInit(LEDCommUARTDriver_t *luartp) {

  luartp->state   = LUART_STOP;
  luartp->txstate = LUART_TX_IDLE;
  luartp->rxstate = LUART_RX_IDLE;
  luartp->config  = NULL;
  /* Optional, user-defined initializer.*/
#if defined(LUART_DRIVER_EXT_INIT_HOOK)
  LUART_DRIVER_EXT_INIT_HOOK(luartp);
#endif
}

/**
 * @brief   Configures and activates the LUART peripheral.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 * @param[in] config    pointer to the @p LEDCommUARTConfig object
 *
 * @api
 */
void luartStart(LEDCommUARTDriver_t *luartp, const LEDCommUARTConfig_t *config) {

  chDbgCheck((luartp != NULL) && (config != NULL), "luartStart");

  chSysLock();
  chDbgAssert((luartp->state == LUART_STOP) || (luartp->state == LUART_READY),
              "luartStart(), #1", "invalid state");

  luartp->config = config;
  // luart_lld_start(luartp);
  luartp->state = LUART_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the LUART peripheral.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 *
 * @api
 */
void luartStop(LEDCommUARTDriver_t *luartp) {

  chDbgCheck(luartp != NULL, "luartStop");

  chSysLock();
  chDbgAssert((luartp->state == LUART_STOP) || (luartp->state == LUART_READY),
              "luartStop(), #1", "invalid state");

  // uart_lld_stop(uartp);
  luartp->state = LUART_STOP;
  luartp->txstate = LUART_TX_IDLE;
  luartp->rxstate = LUART_RX_IDLE;
  chSysUnlock();
}

/**
 * @brief   Starts a transmission on the LUART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
void luartStartSend(LEDCommUARTDriver_t *luartp, size_t n, const void *txbuf) {

  chDbgCheck((luartp != NULL) && (n > 0) && (txbuf != NULL),
             "luartStartSend");
             
  chSysLock();
  chDbgAssert(luartp->state == LUART_READY,
              "luartStartSend(), #1", "is active");
  chDbgAssert(luartp->txstate != LUART_TX_ACTIVE,
              "luartStartSend(), #2", "tx active");

  // uart_lld_start_send(uartp, n, txbuf);
  luartp->txstate = LUART_TX_ACTIVE;
  chSysUnlock();
}

/**
 * @brief   Starts a transmission on the LUART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @iclass
 */
void luartStartSendI(LEDCommUARTDriver_t *luartp, size_t n, const void *txbuf) {

  chDbgCheckClassI();
  chDbgCheck((luartp != NULL) && (n > 0) && (txbuf != NULL),
             "luartStartSendI");
  chDbgAssert(luartp->state == LUART_READY,
              "luartStartSendI(), #1", "is active");
  chDbgAssert(luartp->txstate != LUART_TX_ACTIVE,
              "luartStartSendI(), #2", "tx active");

  // uart_lld_start_send(uartp, n, txbuf);
  luartp->txstate = LUART_TX_ACTIVE;
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 * @retval 0            There was no transmit operation in progress.
 *
 * @api
 */
size_t luartStopSend(LEDCommUARTDriver_t *luartp) {
  size_t n;

  chDbgCheck(luartp != NULL, "luartStopSend");

  chSysLock();
  chDbgAssert(luartp->state == LUART_READY, "luartStopSend(), #1", "not active");

  if (luartp->txstate == LUART_TX_ACTIVE) {
      // n = uart_lld_stop_send(uartp);
      luartp->txstate = LUART_TX_IDLE;
  } else {
      n = 0;
  }
  chSysUnlock();
  return n;
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 * @retval 0            There was no transmit operation in progress.
 *
 * @iclass
 */
size_t luartStopSendI(LEDCommUARTDriver_t *luartp) {

  chDbgCheckClassI();
  chDbgCheck(luartp != NULL, "luartStopSendI");
  chDbgAssert(luartp->state == LUART_READY, "luartStopSendI(), #1", "not active");

  if (luartp->txstate == LUART_TX_ACTIVE) {
      size_t n = 0;
      // TODO:
      luartp->txstate = LUART_TX_IDLE;
      return n;
  }
  return 0;
}

/**
 * @brief   Starts a receive operation on the LUART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] rxbuf     the pointer to the receive buffer
 *
 * @api
 */
void luartStartReceive(LEDCommUARTDriver_t *luartp, size_t n, void *rxbuf) {

  chDbgCheck((luartp != NULL) && (n > 0) && (rxbuf != NULL),
             "luartStartReceive");

  chSysLock();
  chDbgAssert(luartp->state == LUART_READY,
              "luartStartReceive(), #1", "is active");
  chDbgAssert(luartp->rxstate != LUART_RX_ACTIVE,
              "luartStartReceive(), #2", "rx active");

  // uart_lld_start_receive(uartp, n, rxbuf);
  luartp->rxstate = LUART_RX_ACTIVE;
  chSysUnlock();
}

/**
 * @brief   Starts a receive operation on the LUART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
void luartStartReceiveI(LEDCommUARTDriver_t *luartp, size_t n, void *rxbuf) {

  chDbgCheckClassI();
  chDbgCheck((luartp != NULL) && (n > 0) && (rxbuf != NULL),
             "luartStartReceiveI");
  chDbgAssert(luartp->state == LUART_READY,
              "luartStartReceiveI(), #1", "is active");
  chDbgAssert(luartp->rxstate != LUART_RX_ACTIVE,
              "luartStartReceiveI(), #2", "rx active");

  // uart_lld_start_receive(uartp, n, rxbuf);
  luartp->rxstate = LUART_RX_ACTIVE;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] luartp     pointer to the @p LEDCommUARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 * @retval 0            There was no receive operation in progress.
 *
 * @api
 */
size_t luartStopReceive(LEDCommUARTDriver_t *luartp) {
  size_t n;

  chDbgCheck(luartp != NULL, "luartStopReceive");

  chSysLock();
  chDbgAssert(luartp->state == LUART_READY,
              "luartStopReceive(), #1", "not active");

  if (luartp->rxstate == LUART_RX_ACTIVE) {
      // n = uart_lld_stop_receive(uartp);
      luartp->rxstate = LUART_RX_IDLE;
  } else {
      n = 0;
  }
  chSysUnlock();
  return n;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] luartp      pointer to the @p LEDCommUARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 * @retval 0            There was no receive operation in progress.
 *
 * @iclass
 */
size_t luartStopReceiveI(LEDCommUARTDriver_t *luartp) {

  chDbgCheckClassI();
  chDbgCheck(luartp != NULL, "luartStopReceiveI");
  chDbgAssert(luartp->state == LUART_READY,
              "luartStopReceiveI(), #1", "not active");

  if (luartp->rxstate == LUART_RX_ACTIVE) {
      size_t n = 0; // TODO: uart_lld_stop_receive(uartp);
      luartp->rxstate = LUART_RX_IDLE;
      return n;
  }
  return 0;
}

/**
  * @brief   Updates and returns the link status.
  *
  * @param[in] macp      pointer to the @p LEDCommUARTDriver object
  * @return              The link status.
  * @retval TRUE         if the link is active.
  * @retval FALSE        if the link is down.
  *
  * @api
  */
bool_t luartPollLinkStatus(LEDCommUARTDriver_t *luartp) {
    chDbgCheck((luartp != NULL), "luartPollLinkStatus");
    return luartp->link == 1;
}

/** @} */



