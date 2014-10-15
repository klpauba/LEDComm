#include "hal.h"
#include "ledcomm.h"

#if LEDCOMM_THREADED
static BinarySemaphore ledcommINTR;
#endif

inline void ledCommOn(LEDCommDriver_t *ldp) {
    palSetPad(ldp->anode_port, ldp->anode_pad);
    palClearPad(ldp->cathode_port, ldp->cathode_pad);
}

inline void ledCommOff(LEDCommDriver_t *ldp) {
    palClearPad(ldp->anode_port, ldp->anode_pad);
    palClearPad(ldp->cathode_port, ldp->cathode_pad);
}

static inline void ledCommReverse(LEDCommDriver_t *ldp) {
    palClearPad(ldp->anode_port, ldp->anode_pad);
    palSetPad(ldp->cathode_port, ldp->cathode_pad);
}

inline void ledCommInitPad(LEDCommDriver_t *ldp) {
    palSetPadMode((ldp)->anode_port, (ldp)->anode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palSetPadMode((ldp)->cathode_port, (ldp)->cathode_pad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
}

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

uint8_t
computeParity(uint8_t v) {	/* See https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel */
    v ^= v >> 4;
    v &= 0xf;
    return (0x6996 >> v) & 1;
}

uint8_t
ledCommClassify(uint16_t v) {
    switch (ledCommCountTrailingHighBits(v)) {
    case 3: /* 3 - 6 high bits == SPACE, 1 */
    case 4:
    case 5:
    case 6:
	return SPACE;
	break;

    case 7: /* 7 - 10 high bits == MARK, 0 */
    case 8:
    case 9:
    case 10:
	return MARK;
	break;

    case 11: /* 11 - 14 high bits == STOP */
    case 12:
    case 13:
    case 14:
	return STOP;
	break;

    default: /* error */
	return NONE;
	break;
    }
}

static void
ledCommSaveBit(LEDCommDriver_t *ldp) {

    uint8_t bit = ledCommClassify(ldp->rx_register >> 2);	/* shift out two trailing zeros */
    if (isLinkUp(ldp)) {
	if (bit == NONE) {
	    linkDown(ldp);
	} else if (bit == STOP) {
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
	    if (bit == SPACE) {
		ldp->rx_char = ldp->rx_char + 1;
	    }
	}
    } else if (bit == MARK) {
	/* link is down but we received a MARK .. decrement a counter */
	/* used to detect that the link is up */
	ldp->rx_register = ldp->rx_register << 1;	/* shift a MARK bit into the received character register */
	ldp->syncs = ldp->syncs - 1;
	if (ldp->syncs <= 0) {
	    /* The link is UP! */
	    ldp->rx_char = 0;
	    ldp->rx_bits = (ldp->data_bits == LEDCOMM_DATA_BITS7 ? 7 : 8);
	    linkUp(ldp);
	}
    }
}

void
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

void
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
ledCommDispatch(GPTDriver *gptp)		/* Inside a callback: use only i-class functions */
{
    (void)gptp;

#if LEDCOMM_THREADED

    chSysLockFromIsr();
    chBSemResetI(&ledcommINTR, FALSE);
    chSysUnlockFromIsr();

#else
#if LEDCOMM_USE_LCOM1
    ledCommSerialHandler(&LCOM1);
#endif
#if LEDCOMM_USE_LCOM2
    ledCommSerialHandler(&LCOM2);
#endif

#if LEDCOMM_USE_LUART1
    ledCommUARTHandler(&LUART1);
#endif

#if LEDCOMM_USE_LUART
    ledCommUARTHandler(&LUART);
#endif

#endif
}

const    GPTConfig GPTC_LEDComm = {
    .frequency = 10000,		/* 10 kHz */
    .callback = ledCommDispatch,
    0
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

EXTConfig extcfg = {
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

#if LEDCOMM_THREADED
WORKING_AREA(waLEDCommThread, LEDCOMM_THREAD_STACK_SIZE);
msg_t LEDCommThread(void *arg) {

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
	ledCommSerialHandler(&LCOM1);
#endif
#if LEDCOMM_USE_LCOM2
	ledCommSerialHandler(&LCOM2);
#endif
#if LEDCOMM_USE_LUART1
	ledCommUARTHandler(&LUART1);
#endif

#if LEDCOMM_USE_LUART
	ledCommUARTHandler(&LUART);
#endif
    }

    return (msg_t)NULL;
}
#endif

 bool_t ledCommPollLinkStatus(LEDCommDriver_t *ltp) {
    chDbgCheck((ltp != NULL), "luartPollLinkStatus");
    return ltp->link == 1;
 }
