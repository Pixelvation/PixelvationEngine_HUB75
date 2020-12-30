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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_heap_caps.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "rom/lldesc.h"

#include <Arduino.h>

#include "i2s_in.h"

#define APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(x) (((x) + I2S_IN_RAW_NUM_BUFFERS) % I2S_IN_RAW_NUM_BUFFERS)

CircularBuffer i2sInCircularBuffer;
CircularBuffer i2sInRawCircularBuffer;

I2sInRawRxBlock * i2sInRawRxBlocks;
I2sInRxBlock * i2sInRxBlocks;

lldesc_t *i2sInDmaDescriptors;

portMUX_TYPE i2sInCircularBufferMutex = portMUX_INITIALIZER_UNLOCKED;

int numParallelBits;
int i2sParallelMask;

static void gpio_setup_in(gpio_num_t gpio, int sig) {
    if (gpio==-1) return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_matrix_in(gpio, sig, false);
}

static void gpio_setup_in_invert(gpio_num_t gpio, int sig) {
    if (gpio==-1) return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_matrix_in(gpio, sig, true);
}

static void dma_reset(i2s_dev_t *dev) {
    dev->lc_conf.in_rst=1; dev->lc_conf.in_rst=0;
    dev->lc_conf.out_rst=1; dev->lc_conf.out_rst=0;
}

static void fifo_reset(i2s_dev_t *dev) {
    dev->conf.rx_fifo_reset=1; dev->conf.rx_fifo_reset=0;
    dev->conf.tx_fifo_reset=1; dev->conf.tx_fifo_reset=0;
}

inline void i2sRestartWithAddress(uint32_t address) {
    i2s_dev_t *dev = &I2S0;

    dev->in_link.stop=1;

    dev->lc_conf.in_rst=1; dev->lc_conf.out_rst=1; dev->lc_conf.ahbm_rst=1; dev->lc_conf.ahbm_fifo_rst=1;
    dev->lc_conf.in_rst=0; dev->lc_conf.out_rst=0; dev->lc_conf.ahbm_rst=0; dev->lc_conf.ahbm_fifo_rst=0;
    dev->conf.tx_reset=1; dev->conf.tx_fifo_reset=1; dev->conf.rx_fifo_reset=1;
    dev->conf.tx_reset=0; dev->conf.tx_fifo_reset=0; dev->conf.rx_fifo_reset=0;

    dev->in_link.addr=address;
    dev->in_link.start=1;
    dev->conf.rx_start=1;
}

static void i2sInFillFromRaw(bool sendPartial) {
    static int partialBytesStored = 0;

    // only needed for numParallelBits < 8
    static uint8_t partialBits = 0x00;
    static int partialBitsStored = 0;

    int currentI2sInWrite = cbGetNextWrite(&i2sInCircularBuffer);
    int currentI2sInRawRead = cbGetNextRead(&i2sInRawCircularBuffer);

    // data arrives with 8 bits stored in 16 bits, and the order of the 16 bit words is swapped.  Unswap and store the 8-bit data oriented in bytes
    for(int i=0; i<i2sInRawRxBlocks[currentI2sInRawRead].numBytes/2; i++) {
        uint8_t tempByte;

        if (i % 2)
            tempByte = i2sInRawRxBlocks[currentI2sInRawRead].data[(i-1)*2];
        else
            tempByte = i2sInRawRxBlocks[currentI2sInRawRead].data[(i+1)*2];

        if (numParallelBits < 8) {
            partialBits <<= numParallelBits;
            partialBits |= (tempByte & i2sParallelMask);

            partialBitsStored += numParallelBits;
            if (partialBitsStored < 8)
                continue;

            tempByte = partialBits;
            partialBitsStored = 0;
        }

        i2sInRxBlocks[currentI2sInWrite].data[partialBytesStored] = tempByte;
        partialBytesStored++;
    }

    // if we got less than a filled buffer, send the partial data we have
    if(i2sInRawRxBlocks[currentI2sInRawRead].numBytes < I2S_IN_RAW_BUFFER_SIZE)
        sendPartial = true;

    // send if we have a filled buffer
    if(partialBytesStored == I2S_IN_BUFFER_SIZE)
        sendPartial = true;

    if(partialBytesStored > I2S_IN_BUFFER_SIZE){
        printf("partialBytesStored > I2S_IN_BUFFER_SIZE");
        partialBytesStored = 0;
        return;
    }

    if(sendPartial) {
        i2sInRxBlocks[currentI2sInWrite].numBytes = partialBytesStored;
        cbWrite(&i2sInCircularBuffer);
        partialBytesStored = 0;
    }

#ifdef I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
    // testing: fill buffers with known pattern
    memset(&i2sInRawRxBlocks[currentI2sInRawRead], 0x55, sizeof(I2sInRawRxBlock));
#endif

    cbRead(&i2sInRawCircularBuffer);
}

void i2sInPollForNewData() {
    static int lastSeenDescriptorIndex = 0;
    static int lastProcessedDescriptorIndex = 0;
    static unsigned long lastSeenDescriptorChangeTicks = 0;

    //gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);

    // I2S In registers aren't returning what we'd expect.  I2S_INLINK_DSCR_REG should current desc address, but seems like it returns +2.  I2S_INLINK_DSCR_BF0_REG returns +1.  To get current, subtract from I2S_INLINK_DSCR_BF0_REG
    int currentDescriptorIndex = (lldesc_t *)REG_READ(I2S_INLINK_DSCR_BF0_REG(0)) - &i2sInDmaDescriptors[0];
    currentDescriptorIndex = APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(currentDescriptorIndex - 1);

    // write any full blocks received
    while(APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(currentDescriptorIndex - lastProcessedDescriptorIndex) > 0) {
        // this is currently only called from the application, so portENTER_CRITICAL is probably unnecessary, but we may call this from an interrupt context in the future
        portENTER_CRITICAL(&i2sInCircularBufferMutex);
        int currentWrite = cbGetNextWrite(&i2sInRawCircularBuffer);
        uint32_t blockDmaAddress = (uint32_t)(&i2sInDmaDescriptors[lastProcessedDescriptorIndex]);

        // if the CircularBuffer and I2S In descriptors aren't synced up, don't write bad data to CircularBuffer.  Eventually they should sync up
        if(((lldesc_t*)blockDmaAddress)->buf != i2sInRawRxBlocks[currentWrite].data) {
#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS)
         printf("mismatch %d, %08X %08X \n", currentWrite, (uint32_t)((lldesc_t*)blockDmaAddress)->buf, (uint32_t)i2sInRawRxBlocks[currentWrite].data);
#endif
#ifdef DEBUG_I2SIN_3_GPIO
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
#endif
        } else {
            i2sInRawRxBlocks[currentWrite].numBytes = I2S_IN_RAW_BUFFER_SIZE;
            cbWrite(&i2sInRawCircularBuffer);
            i2sInFillFromRaw(false);
        }
        portEXIT_CRITICAL(&i2sInCircularBufferMutex);

        lastProcessedDescriptorIndex = APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(lastProcessedDescriptorIndex+1);
    }

    // if nothing's come in recently
    // TODO: try this with really low clock speed, are we receiving too many partial buffers and breaking something?
    if(currentDescriptorIndex == lastSeenDescriptorIndex) {
        // TODO: is there idle data in the I2S In buffer?
        
        int descriptorPositionMod = 0;
        // check to see if there's any data (smaller than I2sInRawRxBlock) sitting idle, and it's been sitting for a while
        if(descriptorPositionMod && (millis() - lastSeenDescriptorChangeTicks > I2S_IN_PARTIAL_BLOCK_TIMEOUT_MS)) {
#ifdef I2S_IN_GET_PARTIAL_BLOCKS
            // do time sensitive stuff first with interrupts disabled
            portENTER_CRITICAL(&i2sInCircularBufferMutex);

            int currentWrite = cbGetNextWrite(&i2sInRawCircularBuffer);
            uint32_t currentDmaAddress = (uint32_t)(&i2sInDmaDescriptors[currentDescriptorIndex]);
            uint32_t blockDmaAddress = (uint32_t)(&i2sInDmaDescriptors[currentDescriptorIndex - descriptorPositionMod]);
            uint32_t nextDescriptorIndex = (currentDescriptorIndex - descriptorPositionMod + 1) % I2S_IN_RAW_NUM_BUFFERS;

            // move I2S In to beginning of next I2sInRawRxBlock
            uint32_t nextDmaAddress = (uint32_t)(&i2sInDmaDescriptors[nextDescriptorIndex]);

            // if the CircularBuffer and I2S In descriptors aren't synced up, don't write bad data to CircularBuffer, and don't update I2S In.  Eventually they should sync up
            if(((lldesc_t*)blockDmaAddress)->buf == i2sInRawRxBlocks[currentWrite].data) {
                i2sRestartWithAddress(nextDmaAddress);
                i2sInRawRxBlocks[currentWrite].numBytes = descriptorPositionMod * I2S_IN_RAW_BUFFER_SIZE;

                cbWrite(&i2sInRawCircularBuffer);
                i2sInFillFromRaw(true);

                currentDescriptorIndex = nextDescriptorIndex;
                lastSeenDescriptorChangeTicks = millis();

            } else {
                i2sRestartWithAddress((uint32_t)(&i2sInDmaDescriptors[0]));
                cbInit(&i2sInRawCircularBuffer, I2S_IN_RAW_NUM_BUFFERS);
                currentDescriptorIndex = 0;
                lastSeenDescriptorChangeTicks = millis();
            }

            // end of time-sensitive stuff
            portEXIT_CRITICAL(&i2sInCircularBufferMutex);

            lastProcessedDescriptorIndex = nextDescriptorIndex;
#ifdef DEBUG_I2SIN_3_GPIO
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
            gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
#endif
            if(((lldesc_t*)blockDmaAddress)->buf != i2sInRawRxBlocks[currentWrite].data) {
#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS)
                printf("mismatch %d, %08X %08X \n", currentWrite, (uint32_t)((lldesc_t*)blockDmaAddress)->buf, (uint32_t)i2sInRawRxBlocks[currentWrite].data);
#endif
#ifdef DEBUG_I2SIN_3_GPIO
                gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
                gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
#endif
            }

#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_3_DEBUG_PRINTFS)
            printf("nextDmaAddress %08X\n", nextDmaAddress);
            if(((lldesc_t*)nextDmaAddress)->eof)
                printf("EOF: %d\n", ((lldesc_t*)nextDmaAddress)->eof);
#endif

#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_3_DEBUG_PRINTFS)
            // we have data to return
            printf("partial %d %d\n", currentDescriptorIndex, descriptorPositionMod);

            uint8_t * dataPtr = i2sInRawRxBlocks[currentWrite].data;
            for(int i=0; i < I2S_IN_RAW_BUFFER_SIZE; i++) {
                if (i && i%16 == 0)
                    printf("\n");

                printf("%02X ", dataPtr[i]);
            }
            printf("\n");
#endif
#endif
        }
    }

#ifdef DEBUG_I2SIN_3_GPIO
    gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
#endif

    if(lastSeenDescriptorIndex != currentDescriptorIndex) {
        lastSeenDescriptorIndex = currentDescriptorIndex;
        lastSeenDescriptorChangeTicks = millis();
    }
}

void setupI2sIn(int gpioEn, bool risingEdgeClk, int gpioClk, int gpioD0, int gpioD1, int gpioD2, int gpioD3, int gpioD4, int gpioD5, int gpioD6, int gpioD7) {
    i2s_dev_t *dev = &I2S0;

    cbInit(&i2sInRawCircularBuffer, I2S_IN_RAW_NUM_BUFFERS);

    i2sInRawRxBlocks=(I2sInRawRxBlock *)heap_caps_malloc(sizeof(I2sInRawRxBlock) * I2S_IN_RAW_NUM_BUFFERS, MALLOC_CAP_DMA);
    assert("Can't allocate bitplane memory");

    i2sInRxBlocks=(I2sInRxBlock *)malloc(sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS);
    assert("Can't allocate bitplane memory2");

#ifdef I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
    memset(i2sInRawRxBlocks, 0x55, sizeof(I2sInRawRxBlock) * I2S_IN_RAW_NUM_BUFFERS);
    memset(i2sInRxBlocks, 0xAA, sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS);
#endif

    printf("i2sInRawRxBlocks = %08X - %08X\n", (uint32_t)i2sInRawRxBlocks, (uint32_t)i2sInRawRxBlocks+(sizeof(I2sInRawRxBlock) * I2S_IN_RAW_NUM_BUFFERS));
    printf("i2sInRxBlocks = %08X - %08X\n", (uint32_t)i2sInRxBlocks, (uint32_t)i2sInRxBlocks+(sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS));

    printf("Setting up parallel I2S bus for input at I2S0\n");

    int sig_data_base=I2S0I_DATA_IN0_IDX;
    int sig_clk=I2S0I_WS_IN_IDX;

    //Route the signals
    gpio_setup_in(gpioD0, sig_data_base+0);
    numParallelBits = 1;

    if(gpioD1 >= 0) {
        numParallelBits = 2;
        gpio_setup_in(gpioD1, sig_data_base+1);
    }
    if(gpioD2 >= 0) {
        numParallelBits = 3;
        gpio_setup_in(gpioD2, sig_data_base+2);
    }
    if(gpioD3 >= 0) {
        numParallelBits = 4;
        gpio_setup_in(gpioD3, sig_data_base+3);
    }
    if(gpioD4 >= 0) {
        numParallelBits = 5;
        gpio_setup_in(gpioD4, sig_data_base+4);
    }
    if(gpioD5 >= 0) {
        numParallelBits = 6;
        gpio_setup_in(gpioD5, sig_data_base+5);
    }
    if(gpioD6 >= 0) {
        numParallelBits = 7;
        gpio_setup_in(gpioD6, sig_data_base+6);
    }
    if(gpioD7 >= 0) {
        numParallelBits = 8;
        gpio_setup_in(gpioD7, sig_data_base+7);
    }

    // default is data change on rising edge, latch on falling
    if(risingEdgeClk)
        gpio_setup_in_invert(gpioClk, sig_clk);
    else
        gpio_setup_in(gpioClk, sig_clk);

    if(numParallelBits == 1) {
        i2sParallelMask = 0x01;
    } else if (numParallelBits == 2) {
        i2sParallelMask = 0x03;
    } else if (numParallelBits == 4) {
        i2sParallelMask = 0x0F;
    } else if (numParallelBits == 8) {
        i2sParallelMask = 0xFF;
    } else {
        // TODO: print error
    }

// setup debug output
#ifdef DEBUG_I2SIN_1_GPIO
    gpio_pad_select_gpio(DEBUG_I2SIN_1_GPIO);
    gpio_set_direction(DEBUG_I2SIN_1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_I2SIN_1_GPIO, 1);
    gpio_set_level(DEBUG_I2SIN_1_GPIO, 0);
#endif

#ifdef DEBUG_I2SIN_2_GPIO
    gpio_pad_select_gpio(DEBUG_I2SIN_2_GPIO);
    gpio_set_direction(DEBUG_I2SIN_2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
    gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
#endif

#ifdef DEBUG_I2SIN_3_GPIO
    gpio_pad_select_gpio(DEBUG_I2SIN_3_GPIO);
    gpio_set_direction(DEBUG_I2SIN_3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);
    gpio_set_level(DEBUG_I2SIN_3_GPIO, 0);
#endif


    //Initialize I2S dev
    periph_module_enable(PERIPH_I2S0_MODULE);

    // clear settings registers
    dev->conf2.val=0;
    dev->conf_chan.val=0;
    dev->fifo_conf.val=0;
    dev->timing.val=0;

    dev->conf.rx_reset=1; dev->conf.rx_reset=0;
    dev->conf.tx_reset=1; dev->conf.tx_reset=0;
    dma_reset(dev);
    fifo_reset(dev);

    /* Enable camera mode (from Reference Manual): "When I2S is in the camera slave receiving mode,
     * and when I2Sn_H_SYNC, I2S_V_SYNC and I2S_H_REF are held high, the master starts transmitting data"
     * "during data transmission, these three signals should be kept at a high level"
     */
    // TODO: setup I2Sn_H_SYNC, I2S_V_SYNC and I2S_H_REF pins and hold high - seems to work as is though
    gpio_setup_in(gpioEn, I2S0I_H_SYNC_IDX);
    gpio_setup_in(gpioEn, I2S0I_V_SYNC_IDX);
    gpio_setup_in(gpioEn, I2S0I_H_ENABLE_IDX);

    /* Enable camera mode (from Reference Manual): "In order to make I2S work in camera mode,
     * the I2S_LCD_EN bit and the I2S_CAMERA_EN bit of register I2S_CONF2_REG are set to 1,
     * the I2S_RX_SLAVE_MOD bit of register I2S_CONF_REG is set to 1,
     * the I2S_RX_MSB_RIGHT bit and the I2S_RX_RIGHT_FIRST bit of I2S_CONF_REG are set to 0.
     * Thus, I2S works in the LCD slave receiving mode."
     */
    dev->conf2.lcd_en=1;
    dev->conf2.camera_en=1;
    dev->conf.rx_slave_mod=1;
    dev->conf.rx_msb_right=0;
    dev->conf.rx_right_first=0;

    /*
     * "At the same time, in order to use the correct mode to receive data,
     * both the I2S_RX_CHAN_MOD[2:0] bit of register I2S_CONF_CHAN_REG and the I2S_RX_FIFO_MOD[2:0]
     * bit of register I2S_FIFO_CONF_REG are set to 1."
     */
    dev->conf_chan.rx_chan_mod=1;
    dev->fifo_conf.rx_fifo_mod=1;
    
    // enable DMA mode
    dev->fifo_conf.dscr_en=1;

    i2sInDmaDescriptors=(lldesc_t *)heap_caps_malloc(I2S_IN_RAW_NUM_BUFFERS*sizeof(lldesc_t), MALLOC_CAP_DMA);

    printf("i2sInDmaDescriptors = %08X - %08X\n", (uint32_t)i2sInDmaDescriptors, (uint32_t)(i2sInDmaDescriptors+(I2S_IN_RAW_NUM_BUFFERS*sizeof(lldesc_t))));

    for (int i=0; i<I2S_IN_RAW_NUM_BUFFERS; i++) {
        i2sInDmaDescriptors[i].size=I2S_IN_RAW_BUFFER_SIZE;
        i2sInDmaDescriptors[i].length=I2S_IN_RAW_BUFFER_SIZE;
        i2sInDmaDescriptors[i].buf=&(i2sInRawRxBlocks[i].data[0]);
        i2sInDmaDescriptors[i].eof=1;
        i2sInDmaDescriptors[i].sosf=0;
        i2sInDmaDescriptors[i].owner=1;
        i2sInDmaDescriptors[i].qe.stqe_next=(lldesc_t*)&i2sInDmaDescriptors[APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(i+1)];  // link to next buffer, wrapping back to 0 when reached max
        i2sInDmaDescriptors[i].offset=0;
    }

    //Start dma on front buffer
    dev->in_link.addr=((uint32_t)(&i2sInDmaDescriptors[0]));
    dev->in_link.start=1;
    dev->conf.rx_start=1;
}
