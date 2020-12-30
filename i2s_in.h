/*
 * Pixelvation Engine - I2S In
 *
 * Copyright (c) 2020 Louis Beaudoin (Pixelvation)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef I2S_IN_H
#define I2S_IN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "soc/i2s_struct.h"
#include "CircularBuffer.h"

/******** CONFIGURATION *********/
/* 
 * Max size of I2S_IN_RAW_BUFFER_SIZE is DMA_MAX (4096-4), but tt's currently broken for some reason, >256 doesn't work, 
 *   only 128 bytes max (256/2) can be read out, only powers of 2 from 16-256 seem to work, no idea why
 * To workaround these limitations, plus how inefficiently data is stored (1-8 bits of data stored in 16 bits of buffer), and how it's stored in non-sequential order
 *   I2sInRawBuffer receives the data as is from I2S, and then puts it in proper packed order arranged in bytes, and fills i2sInBuffer
 *
 * We want I2S_IN_RAW_BUFFER_SIZE to be large, so it's more efficient to receive a large amount of data, but if I2S_IN_RAW_BUFFER_SIZE is
 *   larger than the the maximum amount of data we can afford to leave idle in the buffer between messages (for APA102 data, larger than the minimum 4 EOF bytes)
 *   then we can't process the message until more data is received later (for APA102 data, this means we're often delayed by 1 frame)
 *   
 * The buffer is polled to see if it's partially full with no changes within a timeout period, then the partial buffer is read, and the I2S hardware pointed to
 *   the start of the next full buffer
 */
#define I2S_IN_RAW_BUFFER_SIZE      256
#define I2S_IN_RAW_NUM_BUFFERS      25

// I2S_IN_BUFFER_SIZE must be an integer multiple of (I2S_IN_RAW_BUFFER_SIZE/2/I2S_IN_PARALLEL_BITS)
#define I2S_IN_BUFFER_SIZE      256
#define I2S_IN_NUM_BUFFERS      25

#define I2S_IN_DEBUG_LEVEL_OFF                  0
#define I2S_IN_DEBUG_LEVEL_1                    1
#define I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS      2
#define I2S_IN_DEBUG_LEVEL_3_DEBUG_PRINTFS      3

#define I2S_IN_DEBUG_LEVEL I2S_IN_DEBUG_LEVEL_OFF
//#define I2S_IN_DEBUG_LEVEL I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS

#define I2S_IN_GET_PARTIAL_BLOCKS
#define I2S_IN_PARTIAL_BLOCK_TIMEOUT_MS     2  

//#define I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
//#define I2S_IN_DEBUG_PRINT_BLOCKS_RECEIVED

// if using, select pins that don't conflict with your hardware
//#define DEBUG_I2SIN_1_GPIO    GPIO_NUM_18
//#define DEBUG_I2SIN_2_GPIO    GPIO_NUM_23
//#define DEBUG_I2SIN_3_GPIO    GPIO_NUM_5

/*********************************/

typedef enum I2sInState {
    empty,
    full,
    overflow,
    dataAvailable,
} I2sInState;

// sizeof() must be multiple of 32-bits as DMA linked list buffer address pointer must be word-aligned.
typedef struct I2sInRxBlock {
    uint8_t     data[I2S_IN_BUFFER_SIZE];
    uint32_t    numBytes;
} I2sInRxBlock;

I2sInRxBlock * i2sInRxBlocks;

typedef struct I2sInRawRxBlock {
    uint8_t     data[I2S_IN_RAW_BUFFER_SIZE];
    uint32_t    numBytes;
} I2sInRawRxBlock;

I2sInRawRxBlock * i2sInRawRxBlocks;

// first four arguments are required, last 7 data pins are optional, use -1 to specify that data pins aren't used
void setupI2sIn(int gpioEn, bool risingEdgeClk, int gpioClk, int gpioD0,
    int gpioD1, int gpioD2, int gpioD3, int gpioD4, int gpioD5, int gpioD6, int gpioD7);
void i2sInPollForNewData(void);

CircularBuffer i2sInCircularBuffer;
CircularBuffer i2sInRawCircularBuffer;

extern portMUX_TYPE i2sInCircularBufferMutex;

#ifdef __cplusplus
}
#endif

#endif
