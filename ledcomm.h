/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/**
 * @file    ledcomm.h
 * @brief   LEDComm Driver macros and structures.
 *
 * @addtogroup LEDComm
 * @{
 */

#ifndef _LEDCOMM_H_
#define _LEDCOMM_H_

#include "ledcommconf.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    LEDComm status flags
 * @{
 */
#define LD_PARITY_ERROR         32	/**< @brief Parity error happened.      */
#define LD_FRAMING_ERROR        64	/**< @brief Framing error happened.     */
#define LD_OVERRUN_ERROR        128	/**< @brief Overflow happened.          */
#define LD_LINK_UP		256	/**< @brief Link detected.              */
#define LD_LINK_DOWN		512	/**< @brief No link detected.           */
/** @} */
/** @} */

#define MARK 0
#define SPACE 1
#define STOP 2
#define NONE 3

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
/**
 * @name    Configuration options
 * @{
 */
#if !defined(LEDCOMM_USE_LCOM1) || defined(__DOXYGEN__)
#define LEDCOMM_USE_LCOM1             TRUE
#endif

#if !defined(LEDCOMM_USE_LCOM2) || defined(__DOXYGEN__)
#define LEDCOMM_USE_LCOM2             FALSE
#endif

#if !defined(LEDCOMM_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define LEDCOMM_BUFFERS_SIZE         16
#endif
/** @} */

#define LEDCOMM_DEFAULT_SYNCS	18	/**< @brief Number of MARKs before considering the link is "up".  */
#define LEDCOMM_DEFAULT_THRESHOLD 3000	/**< @brief Maximum number of hal ticks for a "shine".  */

#define LEDCOMM_DATA_BITS8 0		/**< @brief 8 data bits per character.  */
#define LEDCOMM_DATA_BITS7 1		/**< @brief 7 data bits per character (default).  */
#if !defined(LEDCOMM_DEFAULT_DATA_BITS) || defined(__DOXYGEN__)
#define LEDCOMM_DEFAULT_DATA_BITS      LEDCOMM_DATA_BITS7
#endif

#define LEDCOMM_PARITY_NONE 0		/**< @brief Disable parity generation/detection (default).  */
#define LEDCOMM_PARITY 1		/**< @brief Enable parity generation/detection.  */
#if !defined(LEDCOMM_DEFAULT_PARITY) || defined(__DOXYGEN__)
#define LEDCOMM_DEFAULT_PARITY LEDCOMM_PARITY_NONE
#endif

#define LEDCOMM_PARITY_EVEN 0		/**< @brief Use even parity (default).  */
#define LEDCOMM_PARITY_ODD 1		/**< @brief Use off parity.  */
#define LEDCOMM_PARITY_SPACE 2		/**< @brief Use space parity.  */
#define LEDCOMM_PARITY_MARK 3		/**< @brief Use mark parity.  */
#if !defined(LEDCOMM_DEFAULT_PARITY_TYPE) || defined(__DOXYGEN__)
#define LEDCOMM_DEFAULT_PARITY_TYPE LEDCOMM_PARITY_EVEN
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_USE_QUEUES && !CH_USE_EVENTS
#error "LEDComm Driver requires CH_USE_QUEUES and CH_USE_EVENTS"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   @p LEDCommDriver specific data.
 */
#define _ledcomm_driver_data                                            \
    _base_asynchronous_channel_data					\
    /* Driver state.*/							\
    ldstate_t                 state;					\
    /* Input queue.*/							\
    InputQueue                iqueue;					\
    /* Output queue.*/							\
    OutputQueue               oqueue;					\
    /* Input circular buffer.*/						\
    uint8_t                   ib[LEDCOMM_BUFFERS_SIZE];			\
    /* Output circular buffer.*/					\
    uint8_t                   ob[LEDCOMM_BUFFERS_SIZE];			\
    /* End of the mandatory fields.*/					\
    ioportid_t		anode_port;					\
    ioportmask_t	anode_pad;					\
    ioportid_t		cathode_port;					\
    ioportmask_t	cathode_pad;					\
    uint16_t		threshold;		/* The highest number of elapsed HAL ticks that represents a 'shine' */ \
    uint8_t		syncs:5;		/* The number of MARKs received so far */ \
    uint8_t		data_bits:1;		/* 0=8bits, 1=7bits */  \
    uint8_t		parity:1;		/* 0=no parity, 1=parity */ \
    uint8_t		parity_type:2;		/* 0=even, 1=odd, 2=space, 3=mark  */ \
    uint8_t		parity_flag:1;		/* 1=send/receive parity bit */ \
    uint8_t		enable:1;		/* Enable LED comm */   \
    uint8_t		link:1;			/* 1=link is up */      \
    uint8_t    		txrdy:1;		/* 1=ready to transmit */ \
    uint8_t		rxrdy:1;		/* 1=ready to receive */ \
    uint8_t		txbit:2;		/* 0=MARK, 1=SPACE, 2=STOP */ \
    uint8_t		tx_bits:4;		/* The remaining number of bits to transmit */ \
    uint8_t		rx_bits:4;		/* The remaining number of bits to receive */ \
    uint8_t		tx_char;		/* The character that is being transmitted */ \
    uint8_t		rx_char;		/* The received character */ \
    uint16_t		rx_register;		/* A shift register holding the bits of the character we're currently receiving */ \
    uint32_t		c;			/* The number of HAL ticks at the start of LED detection */ \
    uint8_t		count;			/* A sequence counter */

/**
 * @brief   LEDComm Driver configuration structure.
 * @details An instance of this structure must be passed to @p lcommStart()
 *          in order to configure and start LEDComm driver operations.
 * @note    This structure has some content that is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct {
    ioportid_t		anode_port;             /**< @brief The port where the LED anode is connected. */
    ioportmask_t	anode_pad;		/**< @brief The pad where the LED anode is connected. */
    ioportid_t		cathode_port;		/**< @brief The port where the LED cathode is connected. */
    ioportmask_t	cathode_pad;		/**< @brief The pad where the LED cathode is connected. */
    uint32_t		cathode_extmode;	/**< @brief The processor-specific EXT mode used to configure the extcfg table */
    uint16_t		threshold;		/**< @brief The highest number of elapsed HAL ticks that represents
						 *   a 'shine' -- try @p LEDCOMM_DEFAULT_THRESHOLD (3000)
						 */ 
    uint8_t		data_bits:1;		/**< @brief @p LEDCOMM_DATA_BITS8 or @p LEDCOMM_DATA_BITS7 */
    uint8_t		parity:1;		/**< @brief @p LEDCOMM_PARITY or @p LEDCOMM_PARITY_NONE */
    uint8_t		parity_type:2;		/**< @brief One of @p LEDCOMM_PARITY_EVEN, @p LEDCOMM_PARITY_ODD,
						 *   @p LEDCOMM_PARITY_MARK or @p LEDCOMM_PARITY_SPACE
						 */
    /* End of the mandatory fields.*/
} LEDCommConfig_t;

/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  LD_UNINIT = 0,                    /**< @brief Not initialized.                   */
  LD_STOP = 1,                      /**< @brief Stopped.                           */
  LD_READY = 2                      /**< @brief Ready.                             */
} ldstate_t;

/**
 * @brief   Structure representing a LEDComm driver.
 */
typedef struct LEDCommDriver LEDCommDriver_t;

/**
 * @brief   @p LEDCommDriver specific methods.
 */
#define _ledcomm_driver_methods                                              \
  _base_asynchronous_channel_methods

/**
 * @extends BaseAsynchronousChannelVMT
 *
 * @brief   @p LEDCommDriver virtual methods table.
 */
struct LEDCommDriverVMT {
  _ledcomm_driver_methods
};

/**
 * @extends BaseAsynchronousChannel
 *
 * @brief   Full duplex LEDComm driver class.
 * @details This class extends @p BaseAsynchronousChannel by adding physical
 *          I/O queues.
 */
struct LEDCommDriver {
  /** @brief Virtual Methods Table.*/
  const struct LEDCommDriverVMT *vmt;
  _ledcomm_driver_data
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Direct write to a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chnPutTimeout()
 *
 * @api
 */
#define ldPut(ldp, b) chOQPut(&(ldp)->oqueue, b)

/**
 * @brief   Direct write to a @p LEDCommDriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chnPutTimeout()
 *
 * @api
 */
#define ldPutTimeout(ldp, b, t) chOQPutTimeout(&(ldp)->oqueue, b, t)

/**
 * @brief   Direct read from a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chnGetTimeout()
 *
 * @api
 */
#define ldGet(ldp) chIQGet(&(ldp)->iqueue)

/**
 * @brief   Direct read from a @p LEDCommDriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chnGetTimeout()
 *
 * @api
 */
#define ldGetTimeout(ldp, t) chIQGetTimeout(&(ldp)->iqueue, t)

/**
 * @brief   Direct blocking write to a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @see     chnWrite()
 *
 * @api
 */
#define ldWrite(ldp, b, n)                                                  \
  chOQWriteTimeout(&(ldp)->oqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking write to a @p LEDCommDriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chnWriteTimeout()
 *
 * @api
 */
#define ldWriteTimeout(ldp, b, n, t)                                        \
  chOQWriteTimeout(&(ldp)->oqueue, b, n, t)

/**
 * @brief   Direct non-blocking write to a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chnWriteTimeout()
 *
 * @api
 */
#define ldAsynchronousWrite(ldp, b, n)                                      \
  chOQWriteTimeout(&(ldp)->oqueue, b, n, TIME_IMMEDIATE)

/**
 * @brief   Direct blocking read from a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chnRead()
 *
 * @api
 */
#define ldRead(ldp, b, n)                                                   \
  chIQReadTimeout(&(ldp)->iqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking read from a @p LEDCommDriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chnReadTimeout()
 *
 * @api
 */
#define ldReadTimeout(ldp, b, n, t)                                         \
  chIQReadTimeout(&(ldp)->iqueue, b, n, t)

/**
 * @brief   Direct non-blocking read from a @p LEDCommDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chnReadTimeout()
 *
 * @api
 */
#define ldAsynchronousRead(ldp, b, n)                                       \
  chIQReadTimeout(&(ldp)->iqueue, b, n, TIME_IMMEDIATE)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if LEDCOMM_USE_LCOM1 && !defined(__DOXYGEN__)
extern LEDCommDriver_t LCOM1;
#endif
#if LEDCOMM_USE_LCOM2 && !defined(__DOXYGEN__)
extern LEDCommDriver_t LCOM2;
#endif

#if LEDCOMM_USE_GPTD1 && !defined(__DOXYGEN__)
#define GPTD_LEDComm GPTD1
#endif

#if LEDCOMM_USE_GPTD2 && !defined(__DOXYGEN__)
#define GPTD_LEDComm GPTD2
#endif

#if LEDCOMM_USE_EXTD1 && !defined(__DOXYGEN__)
#define EXTD_LEDComm EXTD1
#endif

#if LEDCOMM_USE_EXTD2 && !defined(__DOXYGEN__)
#define EXTD_LEDComm EXTD2
#endif


#ifdef __cplusplus
extern "C" {
#endif
  void ldInit(void);
  void ldObjectInit(LEDCommDriver_t *ldp, qnotify_t inotify, qnotify_t onotify);
  void ldStart(LEDCommDriver_t *ldp, const LEDCommConfig_t *config);
  void ldStop(LEDCommDriver_t *ldp);
  void ldIncomingDataI(LEDCommDriver_t *ldp, uint8_t b);
  msg_t ldRequestDataI(LEDCommDriver_t *ldp);
#ifdef __cplusplus
}
#endif

#endif /* _LEDCOMM_H_ */

/** @} */
