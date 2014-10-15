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
 * @file    ledcomm.c
 * @brief   LEDComm Driver code (see http://mecrisp.sourceforge.net/ledcomm.htm)
 *
 * @addtogroup LEDComm
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "ledcomm.h"
#include "led.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#if !CH_USE_QUEUES
#error "LEDComm Driver requires CH_USE_QUEUES"
#endif

#if LEDCOMM_USE_LCOM1 || defined(__DOXYGEN__)
LEDCommDriver_t LCOM1;
#endif
#if LEDCOMM_USE_LCOM2 || defined(__DOXYGEN__)
LEDCommDriver_t LCOM2;
#endif

#define LED_SYNC_COUNT 18	/* Number of MARKS required to mark link as up */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*
 * Interface implementation, the following functions just invoke the equivalent
 * queue-level function or macro.
 */

static size_t write(void *ip, const uint8_t *bp, size_t n) {

  return chOQWriteTimeout(&((LEDCommDriver_t *)ip)->oqueue, bp,
                          n, TIME_INFINITE);
}

static size_t read(void *ip, uint8_t *bp, size_t n) {

  return chIQReadTimeout(&((LEDCommDriver_t *)ip)->iqueue, bp,
                         n, TIME_INFINITE);
}

static msg_t put(void *ip, uint8_t b) {

  return chOQPutTimeout(&((LEDCommDriver_t *)ip)->oqueue, b, TIME_INFINITE);
}

static msg_t get(void *ip) {

  return chIQGetTimeout(&((LEDCommDriver_t *)ip)->iqueue, TIME_INFINITE);
}

static msg_t putt(void *ip, uint8_t b, systime_t timeout) {

  return chOQPutTimeout(&((LEDCommDriver_t *)ip)->oqueue, b, timeout);
}

static msg_t gett(void *ip, systime_t timeout) {

  return chIQGetTimeout(&((LEDCommDriver_t *)ip)->iqueue, timeout);
}

static size_t writet(void *ip, const uint8_t *bp, size_t n, systime_t time) {

  return chOQWriteTimeout(&((LEDCommDriver_t *)ip)->oqueue, bp, n, time);
}

static size_t readt(void *ip, uint8_t *bp, size_t n, systime_t time) {

  return chIQReadTimeout(&((LEDCommDriver_t *)ip)->iqueue, bp, n, time);
}

static const struct LEDCommDriverVMT vmt = {
  write, read, put, get,
  putt, gett, writet, readt
};

inline bool_t isLinkUp(LEDCommDriver_t *l) {
    return ledCommPollLinkStatus(l);
}

inline bool_t isLinkDown(LEDCommDriver_t *l) {
    return !ledCommPollLinkStatus(l);
}

inline void linkUp(LEDCommDriver_t *l) {
    l->link = 1;
    l->syncs = LEDCOMM_DEFAULT_SYNCS;
    l->txrdy = 1;
#if CH_USE_EVENTS
#if LEDCOMM_THREADED
    chSysLock();
#else
    chSysLockFromIsr();
#endif
    chIQResetI(&(l->iqueue));
    chOQResetI(&(l->oqueue));
    chnAddFlagsI(l, CHN_CONNECTED);
#if LEDCOMM_THREADED
    chSysUnlock();
#else
    chSysUnlockFromIsr();
#endif
#endif /* CH_USE_EVENTS */
}

inline void linkDown(LEDCommDriver_t *l) {
    l->link = 0;
#if CH_USE_EVENTS
#if LEDCOMM_THREADED
    chSysLock();
#else
    chSysLockFromIsr();
#endif
    chnAddFlagsI(l, CHN_DISCONNECTED);
#if LEDCOMM_THREADED
    chSysUnlock();
#else
    chSysUnlockFromIsr();
#endif
#endif /* CH_USE_EVENTS */
}

/*
 * The default configuration (used if no other is provided).
 */
static LEDCommConfig_t default_config = {
    .anode_port = GPIOB, .anode_pad = 5,
    .cathode_port = GPIOB, .cathode_pad = 4,
    .cathode_extmode = (EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB), /* This is kinda specific to STM32 parts */
    .data_bits = LEDCOMM_DEFAULT_DATA_BITS,
};

void
ledCommSerialHandler(LEDCommDriver_t *ldp)
{
    if (ldp->enable == 0) {
	return;
    }

    ldp->count += 1;

    if (ldp->count == 1) {
	/* Start shining. */
	extChannelDisableI(&EXTD_LEDComm, ldp->cathode_pad);				/* prevent more interrupts */
	palSetPadMode(ldp->cathode_port, ldp->cathode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
	ledCommOn(ldp);
    } else if (ldp->count == 2) {
	if (ldp->txrdy == 1) {
	    msg_t	b;

	    /* Get the next character to transmit. */
#if LEDCOMM_THREADED
	    chSysLock();
#else
	    chSysLockFromIsr();
#endif
	    b = ldRequestDataI(ldp);
#if LEDCOMM_THREADED
	    chSysUnlock();
#else
	    chSysUnlockFromIsr();
#endif

	    if (b < Q_OK) {
		/* Nothing in output queue. */
		ldp->tx_char = 0;
		ldp->txbit = MARK;			/* Send the IDLE pattern */
	    } else {
		ldp->tx_char = (uint8_t)b;
		ldp->txrdy = 0;
	    }
	    ldp->tx_bits = (ldp->data_bits == LEDCOMM_DATA_BITS7 ? 7 : 8);
	}

    } else if (ldp->count == 3) {
	/* decide whether to transmit a MARK, SPACE or STOP bit. */
	if (isLinkDown(ldp) || ldp->txrdy == 1) {
	    /* there is no link or there is nothing to transmit -- we're IDLE, send a MARK */
	    ldp->txbit = MARK;			/* Send the IDLE pattern */
	} else if (ldp->tx_bits == 0) {
	    ldp->txbit = STOP;		/* Send the STOP bit pattern */
	} else {
	    if (ldp->tx_char & (1 << (ldp->tx_bits - 1))) {
		ldp->txbit = SPACE; 		/* Send the "1" bit pattern */
	    } else {
		ldp->txbit = MARK;		/* Send the "0" bit mattern */
	    }
	    ldp->tx_bits = ldp->tx_bits - 1;
	}
    } else if (ldp->count >= 4 && ldp->count <= 13) {
	if ((ldp->count == 5 && ldp->txbit == SPACE) ||
	    (ldp->count == 9 && ldp->txbit == MARK) || 
	    (ldp->count == 13 && ldp->txbit == STOP)) {
	    /* we've transmitted 4 (or 8 or 12) shines: stop shining and start detecting received shines. */
	    ldp->count = 13;	/* start detecting shines */
	    if (ldp->txbit == STOP) {
		ldp->txrdy = 1;
	    }
	    ledCommOff(ldp);
	    ledCommPrepare(ldp);
	}
    } else if (ldp->count >= 14 && ldp->count < 46) { /* DETECT SHINES */
	/* Keep detecting received shines */
	ledCommDetect(ldp);
	ledCommPrepare(ldp);
    } else {
	ledCommDetect(ldp);
	if (ldp->count != 0) {	/* ledCommDetect might have changed ldp->count */
	    /*
	     * There was not a 11100 sequence (otherwise,
	     * the counter would be set to zero in
	     * LEDCommDetect).  We've lost the link.
	     */
	    ldp->count = 0;		/* Maybe set this value to
					   close its maximum value
					   (65,535; with a little
					   randomness) allowing it to
					   roll over to zero.  This
					   will have the effect to
					   dither the start to help
					   prevent occasional
					   sychronization with the
					   other LED?  Note that the
					   logic will have to change a
					   little for this
					   behavior. */
	    if (isLinkUp(ldp)) {
		linkDown(ldp);
	    }
	}
    }
}


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   LEDComm Driver initialization.
 * @note    This function is <B>NOT</B> implicitly invoked by @p halInit(), you must
 *          explicitly initialize the driver.
 *
 * @init
 */
void ldInit(void) {

#if LEDCOMM_USE_LCOM1
    ldObjectInit(&LCOM1, NULL, NULL);
#endif

#if LEDCOMM_USE_LCOM2
    ldObjectInit(&LCOM2, NULL, NULL);
#endif
}

/**
 * @brief   Initializes a generic full duplex driver object.
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] ldp      pointer to a @p LEDCommDriver structure
 * @param[in] inotify   pointer to a callback function that is invoked when
 *                      some data is read from the Queue. The value can be
 *                      @p NULL.
 * @param[in] onotify   pointer to a callback function that is invoked when
 *                      some data is written in the Queue. The value can be
 *                      @p NULL.
 *
 * @init
 */
void ldObjectInit(LEDCommDriver_t *ldp, qnotify_t inotify, qnotify_t onotify) {

  ldp->vmt = &vmt;
  chEvtInit(&ldp->event);
  ldp->state = LD_STOP;
  chIQInit(&ldp->iqueue, ldp->ib, LEDCOMM_BUFFERS_SIZE, inotify, ldp);
  chOQInit(&ldp->oqueue, ldp->ob, LEDCOMM_BUFFERS_SIZE, onotify, ldp);
}

/**
 * @brief   Configures and starts the driver.
 *
 * @param[in] ldp       pointer to a @p LEDCommDriver object
 * @param[in] config    the architecture-dependent LEDComm driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @api
 */
void ldStart(LEDCommDriver_t *ldp, const LEDCommConfig_t *config) {
#if !LEDCOMM_THREADED
    extern GPTConfig GPTC_LEDComm;
#endif
    extern EXTConfig extcfg;
    extern WORKING_AREA(waLEDCommThread, LEDCOMM_THREAD_STACK_SIZE);

    chDbgCheck(ldp != NULL, "ldStart");

    chSysLock();
    chDbgAssert((ldp->state == LD_STOP) || (ldp->state == LD_READY),
		"ldStart(), #1",
		"invalid state");

    if (config == NULL) {
	config = &default_config;
    }

    ldp->anode_port = config->anode_port;
    ldp->anode_pad = config->anode_pad;
    ldp->cathode_port = config->cathode_port;
    ldp->cathode_pad = config->cathode_pad;
    ldp->threshold = config->threshold;
    ldp->syncs = LEDCOMM_DEFAULT_SYNCS;
    ldp->data_bits = config->data_bits;

    ledCommInitPad(ldp);

    extcfg.channels[ldp->cathode_pad].mode = config->cathode_extmode;
    extcfg.channels[ldp->cathode_pad].cb = (extcallback_t)extcb1;

    if (ldp->state == LD_STOP) {
	ldp->enable = 1;
	ldp->link = 0;
	ldp->txrdy = 1;
	ldp->rxrdy = 0;
	ldp->txbit = MARK;
	ldp->tx_bits = (ldp->data_bits == LEDCOMM_DATA_BITS7 ? 7 : 8);
	ldp->tx_char = 0;
	ldp->rx_char = 0;
	ldp->rx_bits = 0;
	ldp->rx_register = 0;
	ldp->c = 0;
	ldp->count = 0;
    }
  
#if LEDCOMM_THREADED
    chSysUnlock();
    chThdCreateStatic(waLEDCommThread, sizeof(waLEDCommThread), HIGHPRIO, LEDCommThread, NULL);
    chSysLock();
#else
    chSysUnlock();
    extStart(&EXTD_LEDComm, &extcfg);

    gptObjectInit(&GPTD_LEDComm);
    gptStart(&GPTD_LEDComm, &GPTC_LEDComm);
    gptStartContinuous(&GPTD_LEDComm, 2);
    chSysLock();
#endif
    ldp->state = LD_READY;
    chSysUnlock();
}

/**
 * @brief   Stops the driver.
 * @details Any thread waiting on the driver's queues will be awakened with
 *          the message @p Q_RESET.
 *
 * @param[in] ldp       pointer to a @p LEDCommDriver object
 *
 * @api
 */
void ldStop(LEDCommDriver_t *ldp) {

  chDbgCheck(ldp != NULL, "ldStop");

  chSysLock();
  chDbgAssert((ldp->state == LD_STOP) || (ldp->state == LD_READY),
              "ldStop(), #1",
              "invalid state");
  ldp->state = LD_STOP;
  chOQResetI(&ldp->oqueue);
  chIQResetI(&ldp->iqueue);
  chSchRescheduleS();
  chSysUnlock();
  extStop(&EXTD_LEDComm);
}

/**
 * @brief   Handles incoming data.
 * @details This function must be called from the input interrupt service
 *          routine in order to enqueue incoming data and generate the
 *          related events.
 * @note    The incoming data event is only generated when the input queue
 *          becomes non-empty.
 * @note    In order to gain some performance it is suggested to not use
 *          this function directly but copy this code directly into the
 *          interrupt service routine.
 *
 * @param[in] ldp       pointer to a @p LEDCommDriver structure
 * @param[in] b         the byte to be written in the driver's Input Queue
 *
 * @iclass
 */
void ldIncomingDataI(LEDCommDriver_t *ldp, uint8_t b) {

  chDbgCheckClassI();
  chDbgCheck(ldp != NULL, "ldIncomingDataI");

#if CH_USE_EVENTS
  if (chIQIsEmptyI(&ldp->iqueue))
    chnAddFlagsI(ldp, CHN_INPUT_AVAILABLE);
#endif

  if (chIQPutI(&ldp->iqueue, b) < Q_OK)
    chnAddFlagsI(ldp, LD_OVERRUN_ERROR);
}

/**
 * @brief   Handles outgoing data.
 * @details Must be called from the output interrupt service routine in order
 *          to get the next byte to be transmitted.
 * @note    In order to gain some performance it is suggested to not use
 *          this function directly but copy this code directly into the
 *          interrupt service routine.
 *
 * @param[in] ldp       pointer to a @p LEDCommDriver structure
 * @return              The byte value read from the driver's output queue.
 * @retval Q_EMPTY      if the queue is empty (the lower driver usually
 *                      disables the interrupt source when this happens).
 *
 * @iclass
 */
msg_t ldRequestDataI(LEDCommDriver_t *ldp) {
  msg_t  b;

  chDbgCheckClassI();
  chDbgCheck(ldp != NULL, "ldRequestDataI");

  b = chOQGetI(&ldp->oqueue);
#if CH_USE_EVENTS
  if (b < Q_OK)
    chnAddFlagsI(ldp, CHN_OUTPUT_EMPTY);
#endif
  return b;
}

/** @} */
