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
 * @file    ledcommconf.h
 * @brief   LEDComm configuration header.
 * @details LEDComm configuration file, this file allows to enable or disable the
 *          various settings for the LEDComm device drivers from your application. You may also use
 *          this file in order to override the device drivers default settings.
 *
 * @defgroup LEDComm_Config LEDComm Configuration
 * @brief    Configuration settings for the LEDComm Driver
 * @{
 */
#ifndef _LEDCOMMCONF_H_
#define _LEDCOMMCONF_H_

/**
 * @brief   Enable LED communication on LCOM1 device if TRUE.
 */
#define LEDCOMM_USE_LCOM1	FALSE

/**
 * @brief   Enable LED communication on LCOM2 device if TRUE.
 */
#define LEDCOMM_USE_LCOM2	TRUE

/**
 * @brief   Use GPT1 for the LEDComm Driver
 * @note    Set only one of <TT>LEDCOMM_USE_GPT1</TT> or <TT>LEDCOMM_USE_GPT2</TT> to TRUE.
 */
#define LEDCOMM_USE_GPTD1	FALSE
/**
 * @brief   Use GPT2 for the LEDComm Driver
 * @note    Set only one of <TT>LEDCOMM_USE_GPT1</TT> or <TT>LEDCOMM_USE_GPT2</TT> to TRUE.
 */
#define LEDCOMM_USE_GPTD2	TRUE

/**
 * @brief   Use EXT1 for the LEDComm Driver.
 * @note    Set only one of <TT>LEDCOMM_USE_EXTD1</TT> or <TT>LEDCOMM_USE_EXTD2</TT> to TRUE.
 */
#define LEDCOMM_USE_EXTD1	TRUE

/**
 * @brief   Use EXT2 for the LEDComm Driver.
 * @note    Set only one of <TT>LEDCOMM_USE_EXTD1</TT> or <TT>LEDCOMM_USE_EXTD2</TT> to TRUE.
 */
#define LEDCOMM_USE_EXTD2	FALSE

/**
 * @brief Use a thread to process the driver logic.
 * @details If set to TRUE, the LEDComm Driver will spend the minimum
 *        time in the ISR and perform most of the processing in a
 *        separate thread.  If set to FALSE, all of the processing
 *        is done in the ISR.
 */
#define LEDCOMM_THREADED	FALSE

/**
 * @brief The size of the input and output queues.
 */
#define LEDCOMM_BUFFERS_SIZE    16

#endif /* _LEDCOMMCONF_H_ */

/** @} */
