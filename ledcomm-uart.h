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
 * @file    ledcomm-uart.h
 * @brief   LEDComm UART Driver macros and structures.
 *
 * @addtogroup UART
 * @{
 */

#ifndef _LEDCOMM_UART_H_
#define _LEDCOMM_UART_H_

#include "ledcommconf.h"

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    LEDComm UART status flags
 * @{
 */
#define LUART_NO_ERROR           0   /**< @brief No pending conditions.      */
#define LUART_PARITY_ERROR       4   /**< @brief Parity error happened.      */
#define LUART_FRAMING_ERROR      8   /**< @brief Framing error happened.     */
#define LUART_OVERRUN_ERROR      16  /**< @brief Overflow happened.          */
#define LUART_NOISE_ERROR        32  /**< @brief Noise on the line.          */
#define LUART_BREAK_DETECTED     64  /**< @brief Break detected.             */
#define LUART_LINK_UP		 256 /**< @brief Link detected.              */
#define LUART_LINK_DOWN		 512 /**< @brief No link detected.           */
/** @} */

#ifndef LEDCOMM_DEFAULT_THRESHOLD
#define LEDCOMM_DEFAULT_THRESHOLD 3000	/**< @brief Maximum number of hal ticks for a "shine".  */
#endif

#ifndef LEDCOMM_DEFAULT_SYNCS
#define LEDCOMM_DEFAULT_SYNCS	18	/**< @brief Number of MARKs before considering the link is "up".  */
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LUART_UNINIT = 0,                  /**< Not initialized.                   */
  LUART_STOP = 1,                    /**< Stopped.                           */
  LUART_READY = 2                    /**< Ready.                             */
} luartstate_t;

/**
 * @brief   Transmitter state machine states.
 */
typedef enum {
  LUART_TX_IDLE = 0,                 /**< Not transmitting.                  */
  LUART_TX_ACTIVE = 1,               /**< Transmitting.                      */
  LUART_TX_COMPLETE = 2,             /**< Buffer complete.                   */
} luarttxstate_t;

/**
 * @brief   Receiver state machine states.
 */
typedef enum {
  LUART_RX_IDLE = 0,                 /**< Not receiving.                     */
  LUART_RX_ACTIVE = 1,               /**< Receiving.                         */
  LUART_RX_COMPLETE = 2,             /**< Buffer complete.                   */
} luartrxstate_t;


/**
 * @brief   UART driver condition flags type.
 */
typedef uint32_t luartflags_t;

/**
 * @brief   Structure representing an UART driver.
 */
typedef struct LEDCommUARTDriver LEDCommUARTDriver_t;

/**
 * @brief   Generic UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
typedef void (*luartcb_t)(LEDCommUARTDriver_t *uartp);

/**
 * @brief   Character received UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] c         received character
 */
typedef void (*luartccb_t)(LEDCommUARTDriver_t *uartp, uint16_t c);

/**
 * @brief   Receive error UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] e         receive error mask
 */
typedef void (*luartecb_t)(LEDCommUARTDriver_t *uartp, luartflags_t e);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
    /**
     * @brief End of transmission buffer callback.
     */
    luartcb_t                  txend1_cb;
    /**
     * @brief Physical end of transmission callback.
     */
    luartcb_t                  txend2_cb;
    /**
     * @brief Receive buffer filled callback.
     */
    luartcb_t                  rxend_cb;
    /**
     * @brief Character received while out if the @p UART_RECEIVE state.
     */
    luartccb_t                 rxchar_cb;
    /**
     * @brief Receive error callback.
     */
    luartecb_t                 rxerr_cb;
    /* End of the mandatory fields.*/

    ioportid_t		anode_port;             /**< @brief The port where the LED anode is connected. */
    ioportmask_t	anode_pad;		/**< @brief The pad where the LED anode is connected. */
    ioportid_t		cathode_port;		/**< @brief The port where the LED cathode is connected. */
    ioportmask_t	cathode_pad;		/**< @brief The pad where the LED cathode is connected. */
    uint32_t		cathode_extmode;	/**< @brief The processor-specific EXT mode used to configure the extcfg table */
    uint16_t		threshold;		/**< @brief The highest number of elapsed HAL ticks that represents
						 *   a 'shine' -- try @p LEDCOMM_DEFAULT_THRESHOLD (3000)
						 */ 
} LEDCommUARTConfig_t;

/**
 * @brief   Structure representing an LEDComm UART driver.
 */
struct LEDCommUARTDriver {
  /**
   * @brief Driver state.
   */
  luartstate_t              state;
  /**
   * @brief Transmitter state.
   */
  luarttxstate_t            txstate;
  /**
   * @brief Receiver state.
   */
  luartrxstate_t            rxstate;
  /**
   * @brief Current configuration data.
   */
  const LEDCommUARTConfig_t *config;
#if defined(UART_DRIVER_EXT_FIELDS)
  LEDCOMM_UART_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/

  EventSource		event;

  /**
   * @brief Default receive buffer while into @p UART_RX_IDLE state.
   */
  uint8_t		link:1;			/* 1=link is up */      \
  volatile uint16_t	rxbuf;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if LEDCOMM_USE_LUART1 && !defined(__DOXYGEN__)
extern LEDCommUARTDriver_t LUART1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void luartInit(void);
    void luartObjectInit(LEDCommUARTDriver_t *uartp);
    void luartStart(LEDCommUARTDriver_t *uartp, const LEDCommUARTConfig_t *config);
    void luartStop(LEDCommUARTDriver_t *uartp);
    void luartStartSend(LEDCommUARTDriver_t *uartp, size_t n, const void *txbuf);
    void luartStartSendI(LEDCommUARTDriver_t *uartp, size_t n, const void *txbuf);
    size_t luartStopSend(LEDCommUARTDriver_t *uartp);
    size_t luartStopSendI(LEDCommUARTDriver_t *uartp);
    void luartStartReceive(LEDCommUARTDriver_t *uartp, size_t n, void *rxbuf);
    void luartStartReceiveI(LEDCommUARTDriver_t *uartp, size_t n, void *rxbuf);
    size_t luartStopReceive(LEDCommUARTDriver_t *uartp);
    size_t luartStopReceiveI(LEDCommUARTDriver_t *uartp);
    bool_t luartPollLinkStatus(LEDCommUARTDriver_t *uartp);
#ifdef __cplusplus
}
#endif

#endif /* _LEDCOMM_UART_H_ */

/** @} */

