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

#define APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(x) (((x) + (I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER)) % (I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER))

CircularBuffer i2sInCircularBuffer;

I2sInRxBlock * i2sInRxBlocks;

lldesc_t *i2sInDmaDescriptors;

portMUX_TYPE i2sInCircularBufferMutex = portMUX_INITIALIZER_UNLOCKED;

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

/****** I2S Shared Code ****/
static void dma_reset(i2s_dev_t *dev) {
    dev->lc_conf.in_rst=1; dev->lc_conf.in_rst=0;
    dev->lc_conf.out_rst=1; dev->lc_conf.out_rst=0;
}

static void fifo_reset(i2s_dev_t *dev) {
    dev->conf.rx_fifo_reset=1; dev->conf.rx_fifo_reset=0;
    dev->conf.tx_fifo_reset=1; dev->conf.tx_fifo_reset=0;
}

static void IRAM_ATTR i2s0_isr(void* arg) {
    uint32_t intStatus = REG_READ(I2S_INT_ST_REG(0));

    if(intStatus) {        
        // Todo: I don't really understand what is being done here, clearing some only if set, and others always?
        REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);

        // clears these:
        // I2S_OUT_TOTAL_EOF_INT_RAW
        // I2S_IN_DSCR_EMPTY_INT_RAW
        // I2S_OUT_DSCR_ERR_INT_RAW
        // I2S_IN_DSCR_ERR_INT_RAW
        // I2S_OUT_EOF_INT_RAW
        // I2S_OUT_DONE_INT_RAW
        // I2S_IN_SUC_EOF_INT_RAW
        // I2S_IN_DONE_INT_RAW
        // I2S_TX_HUNG_INT_RAW
        // I2S_RX_HUNG_INT_RAW

        // doesn't clear these:
        //I2S_TX_REMPTY_INT_RAW The raw interrupt status bit for the I2S_TX_REMPTY_INT interrupt. (RO)
        //I2S_TX_WFULL_INT_RAW The raw interrupt status bit for the I2S_TX_WFULL_INT interrupt. (RO)
        //I2S_RX_REMPTY_INT_RAW The raw interrupt status bit for the I2S_RX_REMPTY_INT interrupt.
        //I2S_RX_WFULL_INT_RAW The raw interrupt status bit for the I2S_RX_WFULL_INT interrupt. (RO)
        //I2S_TX_PUT_DATA_INT_RAW The raw interrupt status bit for the I2S_TX_PUT_DATA_INT interrupt
        //I2S_RX_TAKE_DATA_INT_RAW The raw interrupt status bit for the I2S_RX_TAKE_DATA_INT interrupt

        // this is the only thing enabled: I2S_IN_DONE_INT_ENA

        if(intStatus & I2S_IN_SUC_EOF_INT_ST) {
#if 0
            // sometimes we get this interrupt unexpectedly, sanity check the descripter that is triggering EOF before acting on it
            // Note this doesn't fully fix the issue, as the EOF descriptor may be wrong, but there may be a valid block received.  Need to keep track of descriptor addresses/indexes and compare to see if there's a full block to receive
            int eofDescriptorIndex = (lldesc_t *)REG_READ(I2S_IN_EOF_DES_ADDR_REG(0)) - &i2sInDmaDescriptors[0];
            if((eofDescriptorIndex % I2S_IN_BUFFER_DIVIDER)==(I2S_IN_BUFFER_DIVIDER-1)) {
                i2sInRxBlocks[cbGetNextWrite(&i2sInCircularBuffer)].numBytes = I2S_IN_BUFFER_SIZE;
                cbWrite(&i2sInCircularBuffer);
            }
#else
            i2sInRxBlocks[cbGetNextWrite(&i2sInCircularBuffer)].numBytes = I2S_IN_BUFFER_SIZE;
            cbWrite(&i2sInCircularBuffer);
#endif
        }
    }
}
/****** ************ ****/

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

void i2sInPollForNewData() {
    static int lastSeenDescriptorIndex = 0;
    static int lastProcessedDescriptorIndex = 0;
    static unsigned long lastSeenDescriptorChangeTicks = 0;

    //gpio_set_level(DEBUG_I2SIN_3_GPIO, 1);

    // I2S In registers aren't returning what we'd expect.  I2S_INLINK_DSCR_REG should current desc address, but seems like it returns +2.  I2S_INLINK_DSCR_BF0_REG returns +1.  To get current, subtract from I2S_INLINK_DSCR_BF0_REG
    int currentDescriptorIndex = (lldesc_t *)REG_READ(I2S_INLINK_DSCR_BF0_REG(0)) - &i2sInDmaDescriptors[0];
    currentDescriptorIndex = APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(currentDescriptorIndex - 1);

#if (I2S_IN_EOF_MODE == I2S_IN_EOF_MODE_POLLING)
    // write any full blocks received
    while(APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(currentDescriptorIndex - lastProcessedDescriptorIndex) >= I2S_IN_BUFFER_DIVIDER) {
        // this is currently mutually exclusize with I2S_IN_EOF_MODE_INTERRUPTS mode, and only called from the application, so portENTER_CRITICAL is probably unnecessary, but we may call this from an interrupt context in the future
        portENTER_CRITICAL(&i2sInCircularBufferMutex);
        int currentWrite = cbGetNextWrite(&i2sInCircularBuffer);
        uint32_t blockDmaAddress = (uint32_t)(&i2sInDmaDescriptors[lastProcessedDescriptorIndex]);

        // if the CircularBuffer and I2S In descriptors aren't synced up, don't write bad data to CircularBuffer.  Eventually they should sync up
        if(((lldesc_t*)blockDmaAddress)->buf != i2sInRxBlocks[currentWrite].data) {
#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS)
         printf("mismatch %d, %08X %08X \n", currentWrite, (uint32_t)((lldesc_t*)blockDmaAddress)->buf, (uint32_t)i2sInRxBlocks[currentWrite].data);
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
            i2sInRxBlocks[currentWrite].numBytes = I2S_IN_BUFFER_SIZE;
            cbWrite(&i2sInCircularBuffer);
        }
        portEXIT_CRITICAL(&i2sInCircularBufferMutex);

        lastProcessedDescriptorIndex = APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(lastProcessedDescriptorIndex + I2S_IN_BUFFER_DIVIDER);
    }
#endif

    // if nothing's come in recently
    // TODO: try this with really low clock speed, are we receiving too many partial buffers and breaking something?
    if(currentDescriptorIndex == lastSeenDescriptorIndex) {
        int descriptorPositionMod = currentDescriptorIndex % I2S_IN_BUFFER_DIVIDER;
        // check to see if there's any data (smaller than I2sInRxBlock) sitting idle, and it's been sitting for a while
        if(descriptorPositionMod && (millis() - lastSeenDescriptorChangeTicks > I2S_IN_PARTIAL_BLOCK_TIMEOUT_MS)) {
#ifdef I2S_IN_GET_PARTIAL_BLOCKS
            // do time sensitive stuff first with interrupts disabled
            portENTER_CRITICAL(&i2sInCircularBufferMutex);

            int currentWrite = cbGetNextWrite(&i2sInCircularBuffer);
            uint32_t currentDmaAddress = (uint32_t)(&i2sInDmaDescriptors[currentDescriptorIndex]);
            uint32_t blockDmaAddress = (uint32_t)(&i2sInDmaDescriptors[currentDescriptorIndex - descriptorPositionMod]);
            uint32_t nextDescriptorIndex = ((currentDescriptorIndex - descriptorPositionMod) + I2S_IN_BUFFER_DIVIDER) % (I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER);

            // move I2S In to beginning of next I2sInRxBlock
            uint32_t nextDmaAddress = (uint32_t)(&i2sInDmaDescriptors[nextDescriptorIndex]);

            // if the CircularBuffer and I2S In descriptors aren't synced up, don't write bad data to CircularBuffer, and don't update I2S In.  Eventually they should sync up
            if(((lldesc_t*)blockDmaAddress)->buf == i2sInRxBlocks[currentWrite].data) {
                i2sRestartWithAddress(nextDmaAddress);
                i2sInRxBlocks[currentWrite].numBytes = descriptorPositionMod * (I2S_IN_BUFFER_SIZE / I2S_IN_BUFFER_DIVIDER);
                cbWrite(&i2sInCircularBuffer);
                currentDescriptorIndex = nextDescriptorIndex;
                lastSeenDescriptorChangeTicks = millis();

            } else {
                i2sRestartWithAddress((uint32_t)(&i2sInDmaDescriptors[0]));
                cbInit(&i2sInCircularBuffer, I2S_IN_NUM_BUFFERS);
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
            if(((lldesc_t*)blockDmaAddress)->buf != i2sInRxBlocks[currentWrite].data) {
#if (I2S_IN_DEBUG_LEVEL == I2S_IN_DEBUG_LEVEL_2_ERROR_PRINTFS)
                printf("mismatch %d, %08X %08X \n", currentWrite, (uint32_t)((lldesc_t*)blockDmaAddress)->buf, (uint32_t)i2sInRxBlocks[currentWrite].data);
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

            uint8_t * dataPtr = i2sInRxBlocks[currentWrite].data;
            for(int i=0; i < I2S_IN_BUFFER_SIZE; i++) {
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

/****** I2S In Code ****/
void setupI2sIn(void) {
    i2s_dev_t *dev = &I2S0;

    i2sInRxBlocks=(I2sInRxBlock *)heap_caps_malloc(sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS, MALLOC_CAP_DMA);
    assert("Can't allocate bitplane memory");

#ifdef I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
    memset(i2sInRxBlocks, 0x55, sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS);
#endif

    printf("i2sInRxBlocks = %08X - %08X\n", (uint32_t)i2sInRxBlocks, (uint32_t)i2sInRxBlocks+(sizeof(I2sInRxBlock) * I2S_IN_NUM_BUFFERS));

    printf("Setting up parallel I2S bus for input at I2S0\n");

    int sig_data_base=I2S0I_DATA_IN0_IDX;
    int sig_clk=I2S0I_WS_IN_IDX;

    //Route the signals
    gpio_setup_in(GPIO_I2S_IN_PIN_D0, sig_data_base+0);

#if (I2S_IN_PARALLEL_BITS >= 2)
        gpio_setup_in(GPIO_I2S_IN_PIN_D1, sig_data_base+1);
#endif
#if (I2S_IN_PARALLEL_BITS >= 4)
        gpio_setup_in(GPIO_I2S_IN_PIN_D2, sig_data_base+2);
        gpio_setup_in(GPIO_I2S_IN_PIN_D3, sig_data_base+3);
#endif
#if (I2S_IN_PARALLEL_BITS == 8)
        gpio_setup_in(GPIO_I2S_IN_PIN_D4, sig_data_base+4);
        gpio_setup_in(GPIO_I2S_IN_PIN_D5, sig_data_base+5);
        gpio_setup_in(GPIO_I2S_IN_PIN_D6, sig_data_base+6);
        gpio_setup_in(GPIO_I2S_IN_PIN_D7, sig_data_base+7);
#endif

    // default is data change on rising edge, latch on falling
    if(I2S_INVERT_CLK_SIGNAL)
        gpio_setup_in_invert(GPIO_I2S_IN_PIN_CLK, sig_clk);
    else
        gpio_setup_in(GPIO_I2S_IN_PIN_CLK, sig_clk);

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

    periph_module_enable(PERIPH_I2S0_MODULE);

    //Initialize I2S dev
    dev->conf.rx_reset=1; dev->conf.rx_reset=0;
    dev->conf.tx_reset=1; dev->conf.tx_reset=0;
    dma_reset(dev);
    fifo_reset(dev);

    // TODO: setup I2Sn_H_SYNC, I2S_V_SYNC and I2S_H_REF pins and hold high
    gpio_setup_in(GPIO_I2S_IN_PIN_EN, I2S0I_H_SYNC_IDX);
    gpio_setup_in(GPIO_I2S_IN_PIN_EN, I2S0I_V_SYNC_IDX);
    gpio_setup_in(GPIO_I2S_IN_PIN_EN, I2S0I_H_ENABLE_IDX);
    // enable pullup? external pullup?  eventually input/output to use this pin to reset shift register? GPIO_MODE_INPUT_OUTPUT

    //Enable camera mode
    dev->conf2.val=0;
    dev->conf2.lcd_en=1;
    dev->conf2.camera_en=1;
    dev->conf.rx_slave_mod=1;
    
#if 0
    // Enable "One datum will be written twice in LCD mode" - for some reason, if we don't do this in 8-bit mode, data is updated on half-clocks not clocks
    dev->conf2.lcd_tx_wrx2_en=1;

    dev->sample_rate_conf.val=0;
    dev->sample_rate_conf.rx_bits_mod=8;
    dev->sample_rate_conf.tx_bits_mod=8;
    dev->sample_rate_conf.rx_bck_div_num=2; //ToDo: Unsure about what this does...

    // because conf2.lcd_tx_wrx2_en is set for 8-bit mode, the clock speed is doubled, drop it in half here
    dev->sample_rate_conf.tx_bck_div_num=2;
    
    dev->clkm_conf.val=0;
    dev->clkm_conf.clka_en=0;
    dev->clkm_conf.clkm_div_a=63;
    dev->clkm_conf.clkm_div_b=63;
    //We ignore the possibility for fractional division here.
    //dev->clkm_conf.clkm_div_num=80000000L/cfg->clkspeed_hz;
    dev->clkm_conf.clkm_div_num=3; // datasheet says this must be 2 or greater (but lower values seem to work)
#endif
    dev->fifo_conf.val=0;
    dev->fifo_conf.rx_fifo_mod_force_en=1;
    dev->fifo_conf.tx_fifo_mod_force_en=1;
    //dev->fifo_conf.tx_fifo_mod=1;
    dev->fifo_conf.rx_fifo_mod=1;       // TODO: which value here?  0=32 CLKs, 1=64 CLKs, 2=16CLKs, 3=32CLKs are counted before the 128byte buffer is filled
    dev->fifo_conf.rx_data_num=32; //Thresholds. 
    dev->fifo_conf.tx_data_num=32;
    dev->fifo_conf.dscr_en=1;
    
    dev->conf1.val=0;
    dev->conf1.tx_stop_en=0;
    dev->conf1.rx_pcm_bypass=1;
    
    dev->conf_chan.val=0;
    dev->conf_chan.tx_chan_mod=1;
    dev->conf_chan.rx_chan_mod=1;
    
    //Invert ws to be active-low... ToDo: make this configurable
    //dev->conf.tx_right_first=1;
    dev->conf.tx_right_first=0;
    //dev->conf.rx_right_first=1;
    dev->conf.rx_right_first=0;
    
    dev->timing.val=0;

    i2sInDmaDescriptors=(lldesc_t *)heap_caps_malloc((I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER)*sizeof(lldesc_t), MALLOC_CAP_DMA);

    printf("i2sInDmaDescriptors = %08X - %08X\n", (uint32_t)i2sInDmaDescriptors, (uint32_t)(i2sInDmaDescriptors+((I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER)*sizeof(lldesc_t))));

    // load descriptors so it takes I2S_IN_BUFFER_DIVIDER descriptors to fill one I2sInRxBlock, with an EOF interrupt when each I2sInRxBlock is filled
    for (int i=0; i<(I2S_IN_NUM_BUFFERS * I2S_IN_BUFFER_DIVIDER); i++) {
        i2sInDmaDescriptors[i].size=(I2S_IN_BUFFER_SIZE / I2S_IN_BUFFER_DIVIDER);
        i2sInDmaDescriptors[i].length=(I2S_IN_BUFFER_SIZE / I2S_IN_BUFFER_DIVIDER);
        i2sInDmaDescriptors[i].buf=&(i2sInRxBlocks[(i/I2S_IN_BUFFER_DIVIDER)].data[(i%I2S_IN_BUFFER_DIVIDER)*(I2S_IN_BUFFER_SIZE/I2S_IN_BUFFER_DIVIDER)]);
        i2sInDmaDescriptors[i].eof=(i%I2S_IN_BUFFER_DIVIDER)==(I2S_IN_BUFFER_DIVIDER-1) ? 1 : 0;       // setting EOF for the buffer at the end of each I2sInRxBlock, so there's only an EOF intterupt when each I2sInRxBlock is filled
        i2sInDmaDescriptors[i].sosf=0;
        i2sInDmaDescriptors[i].owner=1;
        i2sInDmaDescriptors[i].qe.stqe_next=(lldesc_t*)&i2sInDmaDescriptors[APPLY_WRAPPING_TO_DESCRIPTOR_INDEX(i+1)];  // link to next buffer, wrapping back to 0 when reached max
        i2sInDmaDescriptors[i].offset=0;
    }

    //Reset FIFO/DMA -> needed? Doesn't dma_reset/fifo_reset do this?
    dev->lc_conf.in_rst=1; dev->lc_conf.out_rst=1; dev->lc_conf.ahbm_rst=1; dev->lc_conf.ahbm_fifo_rst=1;
    dev->lc_conf.in_rst=0; dev->lc_conf.out_rst=0; dev->lc_conf.ahbm_rst=0; dev->lc_conf.ahbm_fifo_rst=0;
    dev->conf.tx_reset=1; dev->conf.tx_fifo_reset=1; dev->conf.rx_fifo_reset=1;
    dev->conf.tx_reset=0; dev->conf.tx_fifo_reset=0; dev->conf.rx_fifo_reset=0;

    //SET_PERI_REG_BITS(I2S_INT_ENA_REG(0), I2S_RX_HUNG_INT_ENA_V, 1, I2S_RX_HUNG_INT_ENA_S);
#if (I2S_IN_EOF_MODE == I2S_IN_EOF_MODE_INTERRUPTS)
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(0), I2S_IN_SUC_EOF_INT_ENA_V, 1, I2S_IN_SUC_EOF_INT_ENA_S);
#endif
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, (int)ESP_INTR_FLAG_IRAM, i2s0_isr, NULL, NULL);

    //Start dma on front buffer
    //dev->lc_conf.val=I2S_INDSCR_BURST_EN;       // TODO: is this the right config?  compare to other I2S Slave Implementations
    dev->in_link.addr=((uint32_t)(&i2sInDmaDescriptors[0]));
    dev->in_link.start=1;
    dev->conf.rx_start=1;
}
/****** ************ ****/

