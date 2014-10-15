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

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "ledcomm.h"
#include "led.h"

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		chThdSleepMilliseconds(500);
	}
	return (msg_t)NULL;
}

BaseSequentialStream *chp;

EVENTSOURCE_DECL(eventSource1);

/*
 * Application entry point.
 */
int main(void) {
    	BaseSequentialStream *lcom2;
	EventListener es1Listener;
	flagsmask_t es1Flags;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);

	/*
	 * Activates the serial driver 2 using the driver default configuration.
	 */
	sdStart(&SD2, NULL);          /* Default is 38400-8-N-1.*/
	chp = (BaseSequentialStream *)&SD2;

	lcom2 = (BaseSequentialStream *)&LCOM2;
	LEDCommConfig_t LCOM2Config = {
	    .anode_port = GPIOB, .anode_pad = 5,
	    .cathode_port = GPIOB, .cathode_pad = 4,
	    .cathode_extmode = (EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB),
	    .threshold = LEDCOMM_DEFAULT_THRESHOLD,
	    .data_bits = (uint8_t)LEDCOMM_DATA_BITS7,
	    .parity = LEDCOMM_PARITY,
	    .parity_type = LEDCOMM_PARITY_EVEN,
	};
	ldInit();
	ldStart(&LCOM2, &LCOM2Config);

	chprintf(chp, "\n\nBeeBoard V1.0\n");
	chprintf(chp, "SYSCLK: %d\n", STM32_SYSCLK);

	/*
	 * Create the threads.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	chEvtRegister((EventSource *)chnGetEventSource((BaseAsynchronousChannel *)lcom2), &es1Listener, EVENT_MASK(1));

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (TRUE) {
	    if (!palReadPad(GPIOC, GPIOC_BUTTON)) {

		/* button contact is now debounced. */
		if (STM32_SYSCLK == 84000000) {
		    chSequentialStreamWrite(lcom2, (const uint8_t *)"F", 1);
		} else {
		    chSequentialStreamWrite(lcom2, (const uint8_t *)"L", 1);
		}
	    }

	    // chprintf(chp, "Waiting on event ...\n");
	    chEvtWaitAllTimeout(EVENT_MASK(1), MS2ST(10));
	    es1Flags = chEvtGetAndClearFlags(&es1Listener);

	    if (es1Flags == 0) {
		continue;
	    }
	    if (es1Flags & CHN_CONNECTED) {
		chprintf(chp, "LINK UP\n");
		palSetPad(GPIOA, GPIOA_LED_GREEN);
		// chSequentialStreamWrite(lcom2, (const uint8_t *)"F401RE", 6);
		// chSequentialStreamWrite(lcom2, (const uint8_t *)"L152RE", 6);
	    }
	    if (es1Flags & CHN_DISCONNECTED) {
		chprintf(chp, "LINK DOWN\n");
		palClearPad(GPIOA, GPIOA_LED_GREEN);
	    }
	    if (es1Flags & CHN_INPUT_AVAILABLE) {
		msg_t rxchar;

		do {
		    rxchar = chnGetTimeout((BaseAsynchronousChannel *)lcom2, TIME_IMMEDIATE); /* get a character from the LED */
		    if (rxchar != Q_TIMEOUT) {
			chprintf(chp, "%c", (uint8_t)rxchar); /* show it */
			chSequentialStreamPut(lcom2, (uint8_t)rxchar); /* echo it back */
		    }
		} while (rxchar != Q_TIMEOUT);
	    }
	    if (es1Flags & LD_OVERRUN_ERROR) {
		chprintf(chp, "OVERRUN\n");
		palClearPad(GPIOA, GPIOA_LED_GREEN);
	    }
	    if (es1Flags & LD_FRAMING_ERROR) {
		chprintf(chp, "FRAMING ERROR\n");
		palClearPad(GPIOA, GPIOA_LED_GREEN);
	    }
	    if (es1Flags & LD_PARITY_ERROR) {
		chprintf(chp, "PARITY ERROR\n");
		palClearPad(GPIOA, GPIOA_LED_GREEN);
	    }
	}
}
