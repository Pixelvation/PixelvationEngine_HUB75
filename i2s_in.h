#ifndef I2S_IN_H
#define I2S_IN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "soc/i2s_struct.h"
#include "CircularBuffer.h"

#define I2S_IN_HARDWARE_THT_PROTOTYPE 1
#define I2S_IN_HARDWARE_SMT_PROTOTYPE 2
#define I2S_IN_HARDWARE_LITE_V0 3

#define I2S_IN_MODE_8BIT_PARALLEL   1
#define I2S_IN_MODE_1BIT_SPI        2

/******** CONFIGURATION *********/
// max size is DMA_MAX (4096-4)
// currently broken somehow, >256 doesn't work, only 128 bytes max (256/2) can be read out, only powers of 2 from 16-256 seem to work, no idea why
#define I2S_IN_BUFFER_SIZE      256

// must be >1 for now
#define I2S_IN_BUFFER_DIVIDER   64

#define I2S_IN_NUM_BUFFERS      25

//#define I2S_IN_MODE I2S_IN_MODE_8BIT_PARALLEL
#define I2S_IN_MODE I2S_IN_MODE_1BIT_SPI

#define I2S_IN_DEBUG_LEVEL_OFF                  0
#define I2S_IN_DEBUG_LEVEL_1                    1
#define I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS      2
#define I2S_IN_DEBUG_LEVEL_3_DEBUG_PRINTFS      3

#define I2S_IN_DEBUG_LEVEL I2S_IN_DEBUG_LEVEL_OFF
//#define I2S_IN_DEBUG_LEVEL I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS

#define I2S_IN_EOF_MODE_INTERRUPTS      1
#define I2S_IN_EOF_MODE_POLLING         2

//#define I2S_IN_EOF_MODE I2S_IN_EOF_MODE_INTERRUPTS                    
#define I2S_IN_EOF_MODE I2S_IN_EOF_MODE_POLLING                    

// Note: currently I2S_IN_GET_PARTIAL_BLOCKS and I2S_IN_EOF_MODE_INTERRUPTS are incompatible
#define I2S_IN_GET_PARTIAL_BLOCKS
#define I2S_IN_PARTIAL_BLOCK_TIMEOUT_MS     2  

//#define I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
//#define I2S_IN_DEBUG_PRINT_BLOCKS_RECEIVED

//#define I2S_IN_HARDWARE I2S_IN_HARDWARE_SMT_PROTOTYPE
//#define I2S_IN_HARDWARE I2S_IN_HARDWARE_THT_PROTOTYPE
#define I2S_IN_HARDWARE I2S_IN_HARDWARE_LITE_V0
/*********************************/

#if (I2S_IN_MODE == I2S_IN_MODE_8BIT_PARALLEL)
    // only support powers of 2 up to 8: 1, 2, 4, 8
    #define I2S_IN_PARALLEL_BITS    8

    // false = use falling edge to clock in signal
    #define I2S_INVERT_CLK_SIGNAL   true

    // IO12 (MTDI), IO15 (MTDO), GPIO0 • GPIO2 • GPIO5 are used for boot strapping, so shouldn't be driven during boot (don't use for I2S In)
    #if (I2S_IN_HARDWARE == I2S_IN_HARDWARE_THT_PROTOTYPE)
        #define GPIO_I2S_IN_PIN_D0  (gpio_num_t)23
        #define GPIO_I2S_IN_PIN_D1  (gpio_num_t)34
        #define GPIO_I2S_IN_PIN_D2  (gpio_num_t)37
        #define GPIO_I2S_IN_PIN_D3  (gpio_num_t)27
        #define GPIO_I2S_IN_PIN_D4  (gpio_num_t)33
        #define GPIO_I2S_IN_PIN_D5  (gpio_num_t)32
        #define GPIO_I2S_IN_PIN_D6  (gpio_num_t)39
        #define GPIO_I2S_IN_PIN_D7  (gpio_num_t)36
        #define GPIO_I2S_IN_PIN_CLK (gpio_num_t)35
        #define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

        #define DEBUG_I2SIN_1_GPIO    GPIO_NUM_9
        #define DEBUG_I2SIN_2_GPIO    GPIO_NUM_14
        //#define DEBUG_I2SIN_3_GPIO    GPIO_NUM_5
    #elif (I2S_IN_HARDWARE == I2S_IN_HARDWARE_SMT_PROTOTYPE)
        #define GPIO_I2S_IN_PIN_D0  (gpio_num_t)33
        #define GPIO_I2S_IN_PIN_D1  (gpio_num_t)32
        #define GPIO_I2S_IN_PIN_D2  (gpio_num_t)35
        #define GPIO_I2S_IN_PIN_D3  (gpio_num_t)34
        #define GPIO_I2S_IN_PIN_D4  (gpio_num_t)39
        #define GPIO_I2S_IN_PIN_D5  (gpio_num_t)38
        #define GPIO_I2S_IN_PIN_D6  (gpio_num_t)37
        #define GPIO_I2S_IN_PIN_D7  (gpio_num_t)36
        #define GPIO_I2S_IN_PIN_CLK (gpio_num_t)21
        #define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

        #define DEBUG_I2SIN_1_GPIO    GPIO_NUM_5
        #define DEBUG_I2SIN_2_GPIO    GPIO_NUM_19
    #endif
#elif (I2S_IN_MODE == I2S_IN_MODE_1BIT_SPI)
    // only support powers of 2 up to 8: 1, 2, 4, 8
    #define I2S_IN_PARALLEL_BITS    1

    // Use rising edge to clock in signal
    #define I2S_INVERT_CLK_SIGNAL   true

    // IO12 (MTDI), IO15 (MTDO), GPIO0 • GPIO2 • GPIO5 are used for boot strapping, so shouldn't be driven during boot (don't use for I2S In)
    #if (I2S_IN_HARDWARE == I2S_IN_HARDWARE_LITE_V0)
        #define GPIO_I2S_IN_PIN_D0  (gpio_num_t)37
        #define GPIO_I2S_IN_PIN_CLK (gpio_num_t)36
        #define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

        #define DEBUG_I2SIN_1_GPIO    GPIO_NUM_18
        #define DEBUG_I2SIN_2_GPIO    GPIO_NUM_23
    #else
        #define GPIO_I2S_IN_PIN_D0  (gpio_num_t)38
        #define GPIO_I2S_IN_PIN_CLK (gpio_num_t)37
        #define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

        #define DEBUG_I2SIN_1_GPIO    GPIO_NUM_5
        #define DEBUG_I2SIN_2_GPIO    GPIO_NUM_19
    #endif
#endif

#define CONVERT_I2SIN_BYTES_TO_DATA_BUFFER_BYTES(i2sInBytes)    (i2sInBytes/2)/(8/I2S_IN_PARALLEL_BITS)

#define DATA_BUFFER_MAX_BYTES   CONVERT_I2SIN_BYTES_TO_DATA_BUFFER_BYTES(I2S_IN_BUFFER_SIZE)

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

void setupI2sIn(void);
void i2sInPollForNewData(void);

CircularBuffer i2sInCircularBuffer;

extern portMUX_TYPE i2sInCircularBufferMutex;

#ifdef __cplusplus
}
#endif

#endif
