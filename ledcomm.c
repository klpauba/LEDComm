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
 * @file    ledcomm.c
 * @brief   LEDComm Driver code (see http://mecrisp.sourceforge.net/ledcomm.htm)
 *
 * @addtogroup LEDComm
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "ledcomm.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#if LEDCOMM_USE_LCOM2 || defined(__DOXYGEN__)
LEDCommDriver_t LCOM2;
#endif

#if LEDCOMM_THREADED
static BinarySemaphore ledcommINTR;
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

static inline void ledCommOn(LEDCommDriver_t *ldp) {
    palSetPad(ldp->anode_port, ldp->anode_pad);
    palClearPad(ldp->cathode_port, ldp->cathode_pad);
}

static inline void ledCommOff(LEDCommDriver_t *ldp) {
    palClearPad(ldp->anode_port, ldp->anode_pad);
    palClearPad(ldp->cathode_port, ldp->cathode_pad);

}

static inline void ledCommReverse(LEDCommDriver_t *ldp) {
    palClearPad(ldp->anode_port, ldp->anode_pad);
    palSetPad(ldp->cathode_port, ldp->cathode_pad);
}

static inline void ledCommInitPad(LEDCommDriver_t *ldp) {
    palSetPadMode((ldp)->anode_port, (ldp)->anode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palSetPadMode((ldp)->cathode_port, (ldp)->cathode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
}

static inline bool_t isLinkUp(LEDCommDriver_t *l) {
    return l->link == 1;	/* 1=link is up */
}

static inline bool_t isLinkDown(LEDCommDriver_t *l) {
    return l->link != 1;	/* 1=link is down */
}

static inline void linkUp(LEDCommDriver_t *l) {
    l->link = 1;
    l->syncs = LEDCOMM_DEFAULT_SYNCS;
    l->rx_char = 0;
    l->rx_bits = (l->data_bits == LEDCOMM_DATA_BITS7 ? 7 : 8);
    l->txrdy = 1;
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
}

static inline void linkDown(LEDCommDriver_t *l) {
    l->link = 0;
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
}

/*
 * The default configuration (used if no other is provided).
 */
static LEDCommConfig_t default_config = {
    .anode_port = GPIOB, .anode_pad = 5,
    .cathode_port = GPIOB, .cathode_pad = 4,
    .cathode_extmode = (EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB), /* This is kinda specific to STM32 parts */
    .data_bits = LEDCOMM_DEFAULT_DATA_BITS,
    .parity = LEDCOMM_DEFAULT_PARITY,
    .parity_type = LEDCOMM_DEFAULT_PARITY_TYPE,
};

void
extcb1(EXTDriver *extp, uint8_t ch) {
    LEDCommDriver_t *ldp;

#if LEDCOMM_USE_LCOM1
    if (ch == LCOM1.cathode_pad) {
	ldp = &LCOM1;
    }
#endif

#if LEDCOMM_USE_LCOM2
    if (ch == LCOM2.cathode_pad) {
	ldp = &LCOM2;
    }
#endif

    extChannelDisableI(extp, ch);				/* prevent more interrupts */

    ldp->c = halGetCounterValue() - ldp->c;
    if (! ((uint32_t)(ldp->c + ldp->threshold) > 2u*ldp->threshold)) {   /* see http://www.microchip.com/forums/m628122.aspx
									    for unsigned comparisons that might roll over. */
	ldp->rx_register = ldp->rx_register + 1;
    }
}

static EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

/* 
 * Count the number of consecutive trailing (right side) high bits using a binary search algorithm.
 */
static uint8_t
ledCommCountTrailingHighBits(uint16_t v)
{
    if (v == 0xffff) {
	/* special case not handled by the code below */
	return 16;
    } else {
	uint8_t c = 0;
	
	if ((v & 0xfff) == 0xfff) {
	    v >>= 12;
	    c += 12;
	}
	if ((v & 0xff) == 0xff) {
	    v >>= 8;
	    c += 8;
	}
	if ((v & 0xf) == 0xf) {
	    v >>= 4;
	    c += 4;
	}
	if ((v & 0x7) == 0x7) {
	    v >>= 3;
	    c += 3;
	}
	if ((v & 0x3) == 0x3) {
	    v >>= 2;
	    c += 2;
	}
	if ((v & 0x1) == 0x1) {
	    v >>= 1;
	    c += 1;
	}

	return c;
    }
}

static uint8_t
computeParity(uint8_t v) {	/* See https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel */
    v ^= v >> 4;
    v &= 0xf;
    return (0x6996 >> v) & 1;
}

static void
ledCommSaveBit(LEDCommDriver_t *ldp) {
    enum { RX_SPACE, RX_MARK, RX_STOP } bit;

    switch (ledCommCountTrailingHighBits(ldp->rx_register >> 2)) {	/* shift out two trailing zeros */
    case 3: /* 3 - 6 high bits == SPACE, 1 */
    case 4:
    case 5:
    case 6:
	bit = RX_SPACE;
	break;

    case 7: /* 7 - 10 high bits == MARK, 0 */
    case 8:
    case 9:
    case 10:
	bit = RX_MARK;
	break;

    case 11: /* 11 - 14 high bits == STOP */
    case 12:
    case 13:
    case 14:
	bit = RX_STOP;
	break;

    default: /* error */
	if (isLinkUp(ldp)) {
	    linkDown(ldp);
	}
	return;
	break;
    }

    if (isLinkUp(ldp)) {
	if (bit == RX_STOP) {
	    uint8_t parity_bit = 0;

	    /* we have a complete character in rx_char */
	    if (ldp->parity) {
		parity_bit = ldp->rx_char & 0x01;
		ldp->rx_char = ldp->rx_char >> 1;
	    }
	    ldp->rx_char = ldp->rx_char & (ldp->data_bits == LEDCOMM_DATA_BITS7 ? 0x7f : 0xff);
	    if (ldp->parity && parity_bit != computeParity(ldp->rx_char)) { /* Check the parity */
#if LEDCOMM_THREADED
		chSysLock();
#else
		chSysLockFromIsr();
#endif
		chnAddFlagsI(ldp, LD_PARITY_ERROR);
#if LEDCOMM_THREADED
		chSysUnlock();
#else
		chSysUnlockFromIsr();
#endif
	    } else {
#if LEDCOMM_THREADED
		chSysLock();
#else
		chSysLockFromIsr();
#endif
		ldIncomingDataI(ldp, ldp->rx_char);
#if LEDCOMM_THREADED
		chSysUnlock();
#else
		chSysUnlockFromIsr();
#endif
	    }
	    ldp->rxrdy = 1;
	    ldp->rx_bits = 0;
	} else {
	    ldp->rx_char = ldp->rx_char << 1;
	    if (bit == RX_SPACE) {
		ldp->rx_char = ldp->rx_char + 1;
	    }
	}
    } else if (bit == RX_MARK) {
	/* link is down but we received a MARK .. decrement a counter */
	/* used to detect that the link is up */
	ldp->rx_register = ldp->rx_register << 1;	/* shift a MARK bit into the received character register */
	ldp->syncs = ldp->syncs - 1;
	if (ldp->syncs <= 0) {
	    /* The link is UP! */
	    linkUp(ldp);
	}
    }
}

static void
ledCommDetect(LEDCommDriver_t *ldp) {
    palClearPad(ldp->cathode_port, ldp->cathode_pad);
    palSetPadMode(ldp->cathode_port, ldp->cathode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);

    if ((ldp->rx_register & 0b11111) == 0b11100) {
	/* got a bit */
	ldp->count = 0;		/* done receiving for this cycle */
	ledCommSaveBit(ldp);
	ldp->rx_register = 0;
    } else {
	ldp->rx_register = ldp->rx_register << 1;
    }
}

static void
ledCommPrepare(LEDCommDriver_t *ldp) {
#if STM32_SYSCLK == 84000000	/* TODO: FIX THIS -- using SYSCLK isn't a great way to do this. */
    #define NOPS 11
#else
    #define NOPS 2
#endif
    uint8_t i = 0;

    /* Forward bias to discharge residual capacitance */
    ledCommOn(ldp);
    for (i = 0; i < NOPS; i++) {
	__NOP();
    }

    /* Reverse bias to charge the parasitic capacitance */
    ledCommReverse(ldp);
    for (i = 0; i < NOPS; i++) {
	__NOP(); __NOP();
	__NOP(); __NOP();
    }

    /* See how long it takes (in "hal ticks") for the capacitance to
       discharge (when the EXT generates an interrupt) */
    ldp->c = halGetCounterValue();	 			/* record number of ticks at the start of the interval */
    extChannelEnableI(&EXTD_LEDComm, ldp->cathode_pad);		/* enable EXT to detect falling edge of cathode */
    palSetPadMode(ldp->cathode_port, ldp->cathode_pad, PAL_MODE_INPUT);
}

static void
ledCommHandler(LEDCommDriver_t *ldp)
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
		ldp->parity_flag = ldp->parity;
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
    } else if (ldp->count == 4) {
	if (ldp->txbit == STOP && ldp->parity_flag) {
	    uint8_t parity_bit = 0;

	    /* Compute the parity bit */
	    ldp->parity_flag = 0;

	    /* Compute the TX parity bit, if needed. */
	    parity_bit = computeParity(ldp->tx_char);

	    if (ldp->parity_type == LEDCOMM_PARITY_EVEN) {
		ldp->txbit = (parity_bit == 0) ? MARK : SPACE;
	    } else if (ldp->parity_type == LEDCOMM_PARITY_ODD) {
		ldp->txbit = (parity_bit == 1) ? MARK : SPACE ;
	    } else if (ldp->parity_type == LEDCOMM_PARITY_MARK) {
		ldp->txbit = MARK;
	    } else if (ldp->parity_type == LEDCOMM_PARITY_SPACE) {
		ldp->txbit = SPACE;
	    } else {
		ldp->tx_bits = 0;
	    }
	}
    } else if (ldp->count >= 5 && ldp->count <= 13) {
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

static void
ledCommDispatch(GPTDriver *gptp)		/* Inside a callback: use only i-class functions */
{
    (void)gptp;

#if LEDCOMM_THREADED

    chSysLockFromIsr();
    chBSemResetI(&ledcommINTR, FALSE);
    chSysUnlockFromIsr();
#else
#if LEDCOMM_USE_LCOM1
    ledCommHandler(&LCOM1);
#endif
#if LEDCOMM_USE_LCOM2
    ledCommHandler(&LCOM2);
#endif
#endif
}

static const    GPTConfig GPTC_LEDComm = {
    .frequency = 10000,		/* 10 kHz */
    .callback = ledCommDispatch,
    0
};

#if LEDCOMM_THREADED
static WORKING_AREA(waLEDCommThread, 512);
static msg_t LEDCommThread(void *arg) {

    (void)arg;
    chRegSetThreadName("LEDComm");

    extStart(&EXTD_LEDComm, &extcfg);

    gptObjectInit(&GPTD_LEDComm);
    gptStart(&GPTD_LEDComm, &GPTC_LEDComm);
    gptStartContinuous(&GPTD_LEDComm, 2);

    chBSemInit(&ledcommINTR, TRUE);
    while (TRUE) {
	chBSemWait(&ledcommINTR);
	chBSemReset(&ledcommINTR, 1);
#if LEDCOMM_USE_LCOM1
	ledCommHandler(&LCOM1);
#endif
#if LEDCOMM_USE_LCOM2
	ledCommHandler(&LCOM2);
#endif
    }

    return (msg_t)NULL;
}
#endif

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
  ldp->parity = config->parity;
  ldp->parity_type = config->parity_type;

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

  if (chIQIsEmptyI(&ldp->iqueue))
    chnAddFlagsI(ldp, CHN_INPUT_AVAILABLE);
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
  if (b < Q_OK)
    chnAddFlagsI(ldp, CHN_OUTPUT_EMPTY);
  return b;
}

/** @} */
