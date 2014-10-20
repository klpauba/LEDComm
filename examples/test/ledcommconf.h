/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ledcommconf.h
 * @brief   LEDComm configuration header.
 * @details LEDComm configuration file, this file allows to enable or disable the
 *          various settings for the LEDComm device drivers from your application. You may also use
 *          this file in order to override the device drivers default settings.
 *
 * @addtogroup LEDComm_Config
 * @{
 */
#ifndef _LEDCOMMCONF_H_
#define _LEDCOMMCONF_H_

/**
 * @brief   LEDComm requires I/O queues.
 */
#define CH_USE_QUEUES                   TRUE

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
 * @brief   The GPT frequency and interval.
 * @note    For compatibility with Mecrisp, use 20000 and 49 (4081 Hz or 2.45 usec period)
 *          Default is 200000 and 40 (5000 Hz or a 2.00 usec period).
 */
#define LEDCOMM_GPT_FREQUENCY	200000
#define LEDCOMM_GPT_INTERVAL    49

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
#define LEDCOMM_THREADED	TRUE

/**
 * @brief The size of the input and output queues.
 */
#define LEDCOMM_BUFFERS_SIZE    16

#endif /* _LEDCOMMCONF_H_ */

/** @} */
