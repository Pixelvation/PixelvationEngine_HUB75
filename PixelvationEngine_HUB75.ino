/*
 * Arduino sketch to receive APA102 data, and drive HUB75 panel using Pixelvation Engine
 * Requires SmartMatrix Library 4.0 and Arduino ESP32
 */

#include <MatrixHardware_ESP32_HUB75AdapterLite_V0.h>    // This file contains multiple ESP32 hardware configurations, edit the file to define GPIOPINOUT (or add #define GPIOPINOUT with a hardcoded number before this #include)
//#include <MatrixHardware_ESP32_HUB75Adapter_SMT.h>
//#include <MatrixHardware_ESP32_HUB75Adapter_THT.h>
#include <SmartMatrix.h>

#include "i2s_in.h"

// colorwheel.c has been modified to use the gimp32x32bitmap struct
#include "colorwheel.c"

#define COLOR_DEPTH 48                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
uint16_t kMatrixWidth = 64;       // Set to the width of your display, must be a multiple of 8
uint16_t kMatrixHeight = 32;      // Set to the height of your display
uint8_t kRefreshDepth = 36;       // Tradeoff of color quality vs refresh rate, max brightness, and RAM usage.  36 is typically good, drop down to 24 if you need to.  On Teensy, multiples of 3, up to 48: 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48.  On ESP32: 24, 36, 48
uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save RAM, more to keep from dropping frames and automatically lowering refresh rate.  (This isn't used on ESP32, leave as default)
uint8_t kPanelType = SM_PANELTYPE_HUB75_32ROW_MOD16SCAN;  // Choose the configuration that matches your panels.  See more details in MatrixCommonHub75.h and the docs: https://github.com/pixelmatix/SmartMatrix/wiki
uint32_t kMatrixOptions = (SM_HUB75_OPTIONS_NONE);        // see docs for options: https://github.com/pixelmatix/SmartMatrix/wiki
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS_NT(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);

#define MODE_TESTING_WITH_SEQUENCE_NUMBERS      1
#define MODE_PARSING_APA102_PROTOCOL            2

#define COLOR_ORDER_RGB 0
#define COLOR_ORDER_RBG 1
#define COLOR_ORDER_GRB 2
#define COLOR_ORDER_GBR 3
#define COLOR_ORDER_BRG 4
#define COLOR_ORDER_BGR 5

#define COLOR_ORDER COLOR_ORDER_RGB

#if (COLOR_ORDER == COLOR_ORDER_RGB)
#define COLOR_ORDER_RED     0
#define COLOR_ORDER_GREEN   1
#define COLOR_ORDER_BLUE    2
#endif
#if (COLOR_ORDER == COLOR_ORDER_RBG)
#define COLOR_ORDER_RED     0
#define COLOR_ORDER_BLUE    1
#define COLOR_ORDER_GREEN   2
#endif
#if (COLOR_ORDER == COLOR_ORDER_GRB)
#define COLOR_ORDER_GREEN   0
#define COLOR_ORDER_RED     1
#define COLOR_ORDER_BLUE    2
#endif
#if (COLOR_ORDER == COLOR_ORDER_GBR)
#define COLOR_ORDER_GREEN   0
#define COLOR_ORDER_BLUE    1
#define COLOR_ORDER_RED     2
#endif
#if (COLOR_ORDER == COLOR_ORDER_BRG)
#define COLOR_ORDER_BLUE    0
#define COLOR_ORDER_RED     1
#define COLOR_ORDER_GREEN   2
#endif
#if (COLOR_ORDER == COLOR_ORDER_BGR)
#define COLOR_ORDER_BLUE    0
#define COLOR_ORDER_GREEN   1
#define COLOR_ORDER_RED     2
#endif

/******** SKETCH OPTIONS *********/
#define SKETCH_MODE MODE_PARSING_APA102_PROTOCOL
//#define SKETCH_MODE MODE_TESTING_WITH_SEQUENCE_NUMBERS

#if (SKETCH_MODE == MODE_TESTING_WITH_SEQUENCE_NUMBERS)
    const uint8_t sequenceCountOrValue = 0x80;
    const int apa102DataBytesPerFrame = (4096) * 4;
    //const int apa102DataBytesPerFrame = 8;
    //const int apa102DataBytesPerFrame = 4096;
    const int apa102DataBytesToSkip = 0;
#elif (SKETCH_MODE == MODE_PARSING_APA102_PROTOCOL)
    const int apa102DataBytesPerFrame = (kMatrixWidth * kMatrixHeight) * 4;
    int testPatternWidth = kMatrixWidth;
    //const int apa102DataBytesPerFrame = (16 * 16) * 4;
    //int testPatternWidth = 16;
    const bool colorCorrectionEnabled = true;
    const bool serpentineLayout = false;
    // apa102DataBytesToSkip is how many bytes (pixels * 4) to ignore before drawing to the buffer (e.g. if two adapters were chained, and we wanted to ignore the pixels going to the first adapter before parsing the later pixels)
    //const int apa102DataBytesToSkip = (128 * 64) * 4;
    const int apa102DataBytesToSkip = 0;
#endif
/*************************/

void drawBitmap(int16_t x, int16_t y, const gimp32x32bitmap* bitmap) {
    for (unsigned int i = 0; i < bitmap->height; i++) {
        for (unsigned int j = 0; j < bitmap->width; j++) {
            SM_RGB pixel = { bitmap->pixel_data[(i * bitmap->width + j) * 3 + 0],
                             bitmap->pixel_data[(i * bitmap->width + j) * 3 + 1],
                             bitmap->pixel_data[(i * bitmap->width + j) * 3 + 2]
                           };

            if(COLOR_DEPTH == 48) {
                pixel.red = pixel.red << 8;
                pixel.green = pixel.green << 8;
                pixel.blue = pixel.blue << 8;
            }

            backgroundLayer.drawPixel(x + j, y + i, pixel);
        }
    }
}

// we store one extra byte than we normally need, as if the incoming data isn't perfectly aligned along the byte boundary, it spills over into one more byte
uint8_t dataBuffer[DATA_BUFFER_MAX_BYTES + 1];


// TODO: do we need this for debugging?
//int previousBufferIndex;

SM_RGB *buffer;
SM_RGB color;

uint8_t gbc;
uint8_t largestGbc;
int x, y;
int framesReceived = 0;
uint32_t * pixelBuffer32bit;

#define I2S_IN_LOCAL_NUM_BUFFERS 212
CircularBuffer i2sInCircularBufferLocal;
I2sInRxBlock ** i2sInRxBlocksLookupTable;

void setup() {
    Serial.begin(115200);
    setupI2sIn();
    cbInit(&i2sInCircularBuffer, I2S_IN_NUM_BUFFERS);

    cbInit(&i2sInCircularBufferLocal, I2S_IN_LOCAL_NUM_BUFFERS);

    matrix.addLayer(&backgroundLayer);

#if (SKETCH_MODE == MODE_PARSING_APA102_PROTOCOL)    
    backgroundLayer.enableColorCorrection(colorCorrectionEnabled);
#endif

    // TODO: use these in some way?
    //matrix.setCalcRefreshRateDivider();
    //matrix.setMaxCalculationCpuPercentage(90);
    matrix.setRefreshRate(240);
#if 1
    matrix.begin();

    matrix.setBrightness(128);

    int x, y;
    backgroundLayer.fillScreen({0, 0, 0});
    x = (kMatrixWidth / 2) - (colorwheel.width / 2);
    y = (kMatrixHeight / 2) - (colorwheel.height / 2);
    // to use drawBitmap, must cast the pointer to pixelmatixlogo as (const gimp32x32bitmap*)
    drawBitmap(x, y, &colorwheel);
    backgroundLayer.swapBuffers(true);
#endif

    printf("32-bit Memory Available: %6d bytes total, %6d bytes largest free block\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));

    // there's only enough remaining RAM for a large pixel buffer in the 32-bit aligned memory space
    pixelBuffer32bit = (uint32_t *)heap_caps_malloc(kMatrixWidth*kMatrixHeight*sizeof(SM_RGB), MALLOC_CAP_32BIT);
    memset(pixelBuffer32bit, 0, kMatrixWidth*kMatrixHeight*sizeof(SM_RGB));

    printf("pixelBuffer32bit = %08X\n", (uint32_t)pixelBuffer32bit);

    printf("32-bit Memory Available: %6d bytes total, %6d bytes largest free block\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));

    // malloc buffer of pointers to I2sInRxBlocks
    i2sInRxBlocksLookupTable=(I2sInRxBlock **)heap_caps_malloc(sizeof(I2sInRxBlock *) * I2S_IN_LOCAL_NUM_BUFFERS, MALLOC_CAP_32BIT);

    printf("i2sInRxBlocksLookupTable = %08X\n", (uint32_t)i2sInRxBlocksLookupTable);

    printf("32-bit Memory Available: %6d bytes total, %6d bytes largest free block\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));

    // malloc I2sInRxBlocks - temporary storage for I2S In data coming from the driver in valuable DMA memory, move to low value 32-bit memory before processing
    for(int i=0; i < I2S_IN_LOCAL_NUM_BUFFERS; i++) {
        i2sInRxBlocksLookupTable[i] = (I2sInRxBlock *)heap_caps_malloc(sizeof(I2sInRxBlock), MALLOC_CAP_32BIT);
        assert("Can't allocate full I2S_IN_LOCAL_NUM_BUFFERS");
    }

    printf("allocated I2S_IN_LOCAL_NUM_BUFFERS = %d\n", I2S_IN_LOCAL_NUM_BUFFERS);

    printf("32-bit Memory Available: %6d bytes total, %6d bytes largest free block\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
}

I2sInState checkStateOfI2sInStream() {
    portENTER_CRITICAL(&i2sInCircularBufferMutex);
    bool isEmpty = cbIsEmpty(&i2sInCircularBuffer);
    portEXIT_CRITICAL(&i2sInCircularBufferMutex);

    if(!isEmpty)
        return dataAvailable;

    portENTER_CRITICAL(&i2sInCircularBufferMutex);
    bool isFull = cbIsFull(&i2sInCircularBuffer);
    bool isOverflow = cbIsOverflow(&i2sInCircularBuffer);
    portEXIT_CRITICAL(&i2sInCircularBufferMutex);

    if(isOverflow)
        return overflow;
    if(isFull)
        return full;

    return empty;
}

void flushI2sInStream() {
    // flush buffer after overflow
    while(!cbIsEmpty(&i2sInCircularBuffer)){
        portENTER_CRITICAL(&i2sInCircularBufferMutex);
        cbRead(&i2sInCircularBuffer);
        portEXIT_CRITICAL(&i2sInCircularBufferMutex);
    }
    
    portENTER_CRITICAL(&i2sInCircularBufferMutex);
    cbClearOverflow(&i2sInCircularBuffer);
    portEXIT_CRITICAL(&i2sInCircularBufferMutex);
}

int getAlignedApa102DataFromI2sInStream(uint8_t * bufferBytes, int bitOffsetLeft, uint8_t & previousBufferPartialByte) {
    I2sInRxBlock * block;

    // TODO: rewrite function to use 32-bit accesses for source data, and avoid temporary copy
    // we deal with bytes, but the buffer is uint32_t oriented (and must be accessed 32-bit aligned if using I2S_IN_PARALLEL_BITS)
    I2sInRxBlock tempBlock;
    int bufferIndexLocal = cbGetNextRead(&i2sInCircularBufferLocal);
    memcpy(&tempBlock, i2sInRxBlocksLookupTable[bufferIndexLocal], sizeof(I2sInRxBlock));
    block = &tempBlock;

    // note this is double the amount of actual data in the buffer due to I2S peripheral quirkiness/misconfiguration
    int i2sInBytesReceived = block->numBytes;

    uint16_t tempWord = 0x0000;
    int dataBufferPosition = 0;

#if (I2S_IN_PARALLEL_BITS < 8)
    int bitsLoadedIntoTempWord = 0;
#endif

#if (I2S_IN_PARALLEL_BITS == 1)
    const int i2sParallelMask = 0x01;
#endif
#if (I2S_IN_PARALLEL_BITS == 2)
    const int i2sParallelMask = 0x03;
#endif
#if (I2S_IN_PARALLEL_BITS == 4)
    const int i2sParallelMask = 0x0F;
#endif
#if (I2S_IN_PARALLEL_BITS == 8)
    const int i2sParallelMask = 0xFF;
#endif

    // load remaining bits from previous buffer into the first byte of the buffer which is only OR'd later
    bufferBytes[0] = previousBufferPartialByte;

    for (int i = 0; i < i2sInBytesReceived/2; i++) {
        // get data of interest out of I2S weird interleaving pattern
        uint8_t tempByte;

        if (i % 2)
            tempByte = block->data[i * 2 - 2];
        else
            tempByte = block->data[i * 2 + 2];

#if (I2S_IN_PARALLEL_BITS < 8)
        tempWord <<= I2S_IN_PARALLEL_BITS;
        tempWord |= (tempByte & i2sParallelMask);

        if (++bitsLoadedIntoTempWord < 8)
            continue;

        bitsLoadedIntoTempWord = 0;
#else
        tempWord = tempByte;
#endif
        tempWord <<= bitOffsetLeft;

        if (bitOffsetLeft) {
            bufferBytes[dataBufferPosition] |= (uint8_t)(tempWord >> 8);
            bufferBytes[dataBufferPosition + 1] = (uint8_t)(tempWord);
        } else {
            bufferBytes[dataBufferPosition] = (uint8_t)tempWord;
            bufferBytes[dataBufferPosition + 1] = 0x00;
        }

        tempWord = 0x00;

        dataBufferPosition++;
    }

#if 0
    printf("shifted\n");

    uint8_t * dataPtr = bufferBytes;
    for(int i=0; i < CONVERT_I2SIN_BYTES_TO_DATA_BUFFER_BYTES(i2sInBytesReceived); i++) {
        if (i && i%16 == 0)
            printf("\n");

        printf("%02X ", dataPtr[i]);
    }
    printf("\n");
#endif
    cbRead(&i2sInCircularBufferLocal);

    // save remaining bits from previous buffer to be parsed along with next received I2sInRxBlock
    previousBufferPartialByte = bufferBytes[CONVERT_I2SIN_BYTES_TO_DATA_BUFFER_BYTES(i2sInBytesReceived)];

    return (CONVERT_I2SIN_BYTES_TO_DATA_BUFFER_BYTES(i2sInBytesReceived));
}

// returns apa102Position:
// - return -1 for don't process current byte, currently waiting for Frame Start Message
// - return n>=0 for SOF received, buffer[i] contains n byte within APA102 message (0=first byte of first LED frame)
int getPositionWithinApa102Frame(uint8_t * bufferBytes, unsigned int dataBufferBytesToProcess, unsigned int currentPosition, int & bitOffset, uint8_t & previousBufferPartialByte) {
    static uint32_t lastFourBytes = 0xFFFFFFFFUL;
    static int apa102Position = -1;
    bool continueWithoutShifting = false;
    int previousBitOffset;

    // this function is looking for *the last instance* of 32 zero bits in a row, indicating an APA102 Frame Start, then aligns the data stream with the Frame Start message, and returns the current position within the APA102 message
    bool stillHopeFor32Zeros = false;

    // if we're in the middle of an APA102 frame, keep counting
    if(apa102Position >= 0)
        apa102Position++;

    // if lastFourBytes has this pattern: 0x??000000, there's a chance that we've recently received >=32 continuous zeros
    if((lastFourBytes & 0x00FFFFFFUL) == 0x00000000UL) {
        stillHopeFor32Zeros = true;
    }

    //printf("\r\nlastFourBytes = 0x000000 ");

    // if we have >= 32 continuous zero bits, but no non-zero bits after, we know we received a Frame Start, but can't yet align our data, so stop trying and wait for the next byte
    if (stillHopeFor32Zeros && bufferBytes[currentPosition] == 0x00) {
        //printf("\r\nall zeros, continue ");
        apa102Position = -1;
        stillHopeFor32Zeros = false;
    }

    if(stillHopeFor32Zeros) {    
        // look for 32 continuous zeros, and specifically *the last instance* of 32 continuous zeros
        uint16_t sideBytes = ((lastFourBytes & 0xFF000000) >> 16) | bufferBytes[currentPosition];
        // we know there are 24 continuous zeros already in the middle bytes, look for the last 8 zero bits on either side
        //printf("\r\nsidebytes: %04X ", sideBytes);

        previousBitOffset = bitOffset;
        stillHopeFor32Zeros = false;

        for (int j = 0; j < 8; j++) {
            //printf("\r\nsidebytes << %d= %04X ", j, sideBytes);
            if ((uint8_t)(sideBytes >> 8) == 0x00) {
                bitOffset = previousBitOffset + j;
                bitOffset %= 8;

                // we found a zero, but not necessarly the last zero, keep going until we know the correct value of bitOffset for aligning on the last zero
                stillHopeFor32Zeros = true;
                //printf("\r\nfoundzero: bitOffset %d ", bitOffset);
            }
            sideBytes <<= 1;
        }
    }

    if (stillHopeFor32Zeros) {
        apa102Position = 0;
        // if we have a new bitOffset, data already buffered needs to include the new shift
        if (bitOffset != previousBitOffset) {
            bool shiftLeftToShiftRight = false;
            int numShifts;

            if (bitOffset < previousBitOffset) {
                shiftLeftToShiftRight = true;
                numShifts = 8 - (previousBitOffset - bitOffset);
            } else {
                numShifts = (bitOffset - previousBitOffset);
            }

            // do left shifts
            lastFourBytes <<= numShifts;
            lastFourBytes |= (bufferBytes[currentPosition] << numShifts) >> 8;

            for (int k = currentPosition; k < dataBufferBytesToProcess; k++) {
                uint16_t tempWord = (bufferBytes[k] << 8) | bufferBytes[k + 1];
                tempWord <<= numShifts;
                bufferBytes[k] = (uint8_t)(tempWord >> 8);
            }

            bufferBytes[dataBufferBytesToProcess] <<= numShifts;

            if (shiftLeftToShiftRight) {
                // copy the data from dataBuffer[i+1] to dataBuffer[dataBufferBytesToProcess], as dataBuffer[dataBufferBytesToProcess] is now empty with a gap in data
                for (int k = currentPosition; k < dataBufferBytesToProcess; k++) {
                    bufferBytes[k + 1] = bufferBytes[k];
                }

                previousBufferPartialByte = bufferBytes[dataBufferBytesToProcess];

                // we shifted all the data right one byte, so bufferBytes[currentPosition] is unusable.  The first byte of first LED frame is in bufferBytes[currentPosition+1].  return without modifying lastFourBytes, and we'll handle bufferBytes[currentPosition+1] on the next pass through
                apa102Position = -1;

                // we can't add bufferBytes[currentPosition] to lastFourBytes as it's garbage data
                continueWithoutShifting = true;
                //printf("++shiftleft++:");
            }

            printf("new: %d, ", bitOffset);
        }

        //printf("\r\nSOF received, first byte: %02X", dataBuffer[i]);
    }

    if(!continueWithoutShifting) {
        lastFourBytes <<= 8;
        lastFourBytes |= bufferBytes[currentPosition];
    }

    return apa102Position;
}

void loop() {
    static int bitOffset = 0;
    static uint8_t lastPartialByte = 0x00;
    static unsigned long lastPrintMs = 0;
    bool i2sInDataToProcess;
    bool i2sInLocalFull;
    I2sInState i2sState;

#if (SKETCH_MODE == MODE_TESTING_WITH_SEQUENCE_NUMBERS)
    static int sequenceCountValue = 0;
#endif
    static bool counting = false;
    static bool countingFail = false;
    static bool runOncePerCountingFail;

    if (millis() - lastPrintMs > 1000) {
        lastPrintMs = millis();

#include "soc/i2s_reg.h"

        printf(" %d, %d, %d \r\n", framesReceived, cbGetMinFree(&i2sInCircularBuffer), cbGetMinFree(&i2sInCircularBufferLocal));
        //printf(" %d, %d, %d %08X %08X %08X %08X %08X\r\n", framesReceived, cbGetMinFree(&i2sInCircularBuffer), cbGetMinFree(&i2sInCircularBufferLocal), REG_READ(I2S_INLINK_DSCR_REG(0)), REG_READ(I2S_INLINK_DSCR_BF0_REG(0)), REG_READ(I2S_IN_EOF_DES_ADDR_REG(0)), REG_READ(I2S_INLINK_DSCR_BF1_REG(0)), REG_READ(I2S_LC_STATE0_REG(0)));
        cbClearMinFree(&i2sInCircularBufferLocal);
        cbClearMinFree(&i2sInCircularBuffer);
    }

    do {
        i2sInPollForNewData();

        i2sInDataToProcess = false;
        i2sInLocalFull = false;
        i2sState = checkStateOfI2sInStream();

        if(i2sState == empty) {
            // do nothing
        } else if(i2sState == full || i2sState == overflow) {
            // TODO: flush local buffers as well?
            flushI2sInStream();
            printf("%%");
            countingFail = true;
            counting = false;
        } else {
            // we must have data to process
            i2sInDataToProcess = true;
        }

        if(cbIsFull(&i2sInCircularBufferLocal))
            i2sInLocalFull = true;

        if(i2sInDataToProcess && !i2sInLocalFull) {
            portENTER_CRITICAL(&i2sInCircularBufferMutex);
            int bufferIndex = cbGetNextRead(&i2sInCircularBuffer);
            portEXIT_CRITICAL(&i2sInCircularBufferMutex);

            int bufferIndexLocal = cbGetNextWrite(&i2sInCircularBufferLocal);

            memcpy(i2sInRxBlocksLookupTable[bufferIndexLocal], &i2sInRxBlocks[bufferIndex], sizeof(I2sInRxBlock));

#ifdef I2S_IN_DEBUG_PRINT_BLOCKS_RECEIVED
            printf("got: %d %d\n", bufferIndex, i2sInRxBlocks[bufferIndex].numBytes);

            uint8_t * dataPtr = i2sInRxBlocks[bufferIndex].data;
            for(int i=0; i < I2S_IN_BUFFER_SIZE; i++) {
                if (i && i%16 == 0)
                    printf("\n");

                printf("%02X ", dataPtr[i]);
            }
            printf("\n");
#endif

#ifdef I2S_IN_DEBUG_FILL_BLOCKS_WITH_PATTERN
            // testing: fill buffers with known pattern
            memset(&i2sInRxBlocks[bufferIndex], 0x55, sizeof(I2sInRxBlock));
#endif

            cbWrite(&i2sInCircularBufferLocal);

            portENTER_CRITICAL(&i2sInCircularBufferMutex);
            cbRead(&i2sInCircularBuffer);
            portEXIT_CRITICAL(&i2sInCircularBufferMutex);
        }
    } while(i2sInDataToProcess && !i2sInLocalFull);

    if(cbIsEmpty(&i2sInCircularBufferLocal))
        return;

    if(cbIsOverflow(&i2sInCircularBufferLocal)) {
        // flush buffer after overflow
        while(!cbIsEmpty(&i2sInCircularBufferLocal)){
            cbRead(&i2sInCircularBufferLocal);
        }
        
        cbClearOverflow(&i2sInCircularBufferLocal);

        flushI2sInStream();
        printf("^^");
        countingFail = true;
        counting = false;
        return;
    }

#ifdef DEBUG_I2SIN_2_GPIO
        gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
        gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
        gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
#endif

        // TODO: read number of bytes and only process that amount
        int dataBufferBytesToProcess = getAlignedApa102DataFromI2sInStream(dataBuffer, bitOffset, lastPartialByte);

        for (int i = 0; i < dataBufferBytesToProcess; i++) {
            if (!(i % 16)) {
                //printf("\r\n offset: %d | ", bitOffset);
            }
            //printf("%02X ", dataBuffer[i]);

            int apa102Position = getPositionWithinApa102Frame(dataBuffer, dataBufferBytesToProcess, i, bitOffset, lastPartialByte);

            if(apa102Position < 0)
                continue;

            // SOF was just received 
            if(apa102Position == 0) {
                counting = true;
                runOncePerCountingFail = true;
                countingFail = false;
                x = 0;
                y = 0;
                largestGbc = 0;

#if (SKETCH_MODE == MODE_TESTING_WITH_SEQUENCE_NUMBERS)
                sequenceCountValue = sequenceCountOrValue | 0x01;
#endif
            }

            // keep track of data inside SOF and EOF markers, make sure it stays in sequence and isn't too long or too short
            if (counting) {
#if 0
                if (apa102Position == 0) {
                    printf("\r\n offset: %d | %02X ", bitOffset, dataBuffer[i]);
                }
                if (apa102Position == 1) {
                    printf(",%02X ", dataBuffer[i]);
                }
#endif
#if (SKETCH_MODE == MODE_TESTING_WITH_SEQUENCE_NUMBERS)
                if (dataBuffer[i] == sequenceCountValue) {
                    //printf(".");
                } else {
                    if (runOncePerCountingFail) {
#if 1
                        printf("-");
                        runOncePerCountingFail = false;
#else
                        gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
                        gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
                        gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
                        gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
                        printf("\r\n- exp: %02X, got: %02X ", sequenceCountValue, dataBuffer[i]);
                        //runOncePerCountingFail = false;
#endif
                        countingFail = true;
                    }
                }

                sequenceCountValue = dataBuffer[i];
                sequenceCountValue++;

                // skip invalid test values of 0x00 and 0xFF
                if (sequenceCountValue == 0xFF) {
                    //printf(".");
                    sequenceCountValue = sequenceCountOrValue | 0x01;
                }

#elif (SKETCH_MODE == MODE_PARSING_APA102_PROTOCOL)
                if(apa102Position >= apa102DataBytesToSkip) {
                    if (apa102Position % 4 == 0) {
                        gbc = dataBuffer[i] & 0x1F;
                        if(gbc > largestGbc)
                            largestGbc = gbc;

                        if ((dataBuffer[i] & 0xE0) != 0xE0) {
                            if (runOncePerCountingFail) {
                                printf("-%02X:%d:%02X:%02X:%02X:%02X,", dataBuffer[i], apa102Position, dataBuffer[i + 1], dataBuffer[i + 2], dataBuffer[i + 3], dataBuffer[i + 4]);
                                runOncePerCountingFail = false;
                            }
                            countingFail = true;
                            counting = false;
                            continue;
                        }
                    } else {
                        uint16_t tempIntensity = ((((uint16_t)dataBuffer[i] * gbc) + (gbc >> 1)) << 8 ) / 0x20;

                        if(COLOR_DEPTH == 24)
                            tempIntensity >>= 8;

                        if (apa102Position % 4 == 1) {
#if (COLOR_ORDER_RED == 0)                            
                            color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 0)                            
                            color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 0)                            
                            color.blue = tempIntensity;
#endif
                        }
                        if (apa102Position % 4 == 2) {
#if (COLOR_ORDER_RED == 1)                            
                            color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 1)                            
                            color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 1)                            
                            color.blue = tempIntensity;
#endif
                        }
                        if (apa102Position % 4 == 3) {
#if (COLOR_ORDER_RED == 2)                            
                            color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 2)                            
                            color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 2)                            
                            color.blue = tempIntensity;
#endif

                            static SM_RGB previousPixel;

#if (COLOR_DEPTH == 48)
                            // store pixels temporarily into pixelBuffer32bit (32-bit aligned access only), storing two pixels at once (2x48 is multiple of 32 bits)
                            if(x%2 == 0) {
                                previousPixel = color;
                            } else {
                                int bigBufferOffset;
                                if(serpentineLayout && y%2) {
                                    bigBufferOffset = (sizeof(SM_RGB) * (((testPatternWidth - 1) - x) + (y * kMatrixWidth))) / sizeof(uint32_t);
    
                                    pixelBuffer32bit[bigBufferOffset + 0] = (color.blue << 16) | (color.red << 0);
                                    pixelBuffer32bit[bigBufferOffset + 1] = (previousPixel.red << 16) |  (color.green << 0);
                                    pixelBuffer32bit[bigBufferOffset + 2] = (previousPixel.green << 16) | (previousPixel.blue << 0);
                                } else {
                                    bigBufferOffset = (sizeof(SM_RGB) * ((x-1) + (y * kMatrixWidth))) / sizeof(uint32_t);
    
                                    pixelBuffer32bit[bigBufferOffset + 0] = (previousPixel.blue << 16) | (previousPixel.red << 0);
                                    pixelBuffer32bit[bigBufferOffset + 1] = (color.red << 16) |  (previousPixel.green << 0);
                                    pixelBuffer32bit[bigBufferOffset + 2] = (color.green << 16) | (color.blue << 0);
                                }
                            }
#else   // COLOR_DEPTH == 24
                            // not yet supported
#endif
                            x++;
                            if (x >= testPatternWidth) {
                                x = 0;
                                y++;
                            }
                        }
                    }
                }   
#endif

                if (apa102Position == apa102DataBytesPerFrame + apa102DataBytesToSkip - 1) {
                    counting = false;
                    if (!countingFail) {
                        framesReceived++;

#if (SKETCH_MODE == MODE_PARSING_APA102_PROTOCOL)
                        if (!backgroundLayer.isSwapPending()){
                            // copy pixels from temporary buffer (note: 32-bit aligned) to backgroundLayer
                            buffer = backgroundLayer.backBuffer();
                            memcpy(buffer, pixelBuffer32bit, kMatrixWidth*kMatrixHeight*sizeof(SM_RGB));

                            // scaling fails if largestGbc==0 (all pixels were black)
                            if(!largestGbc)
                                largestGbc = 0x1F;

                            // figure out how many times we can double background layer brightness without exceeding the max
                            int numShifts=0;
                            while(!(largestGbc / (0x01 << (4-numShifts))))
                                numShifts++;

                            backgroundLayer.setBrightnessShifts(numShifts);
                            backgroundLayer.swapBuffers(false);
                        }

                        //matrix.countFPS();
#endif
                    }
                }
            }
        }

        gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
        gpio_set_level(DEBUG_I2SIN_2_GPIO, 1);
        gpio_set_level(DEBUG_I2SIN_2_GPIO, 0);
        //printf("\r\n\r\nDATA BUFFER PROCESSED %d\r\n", bufferIndex);
#if 0
    // TODO: this error is seldom seen, what is it?  Do we need to track it somewhere?
        if (previousBufferIndex == bufferIndex) {
            printf("\r\n!!!!\r\n");
        }
        //printf("\r\n%d ", bufferIndex);
        previousBufferIndex = bufferIndex;
#endif
}
