/* CircularBuffer.cpp
modified from example code from Wikipedia:
http://en.wikipedia.org/wiki/Circular_queue#Use_a_Fill_Count

modified to only contain indexes and not any data elements, and allow for peeking at the next read/write index
*/

#include "CircularBuffer.h"


void cbInit(CircularBuffer *cb, int size) {
    cb->size  = size;
    cb->start = 0;
    cb->count = 0;
    cb->min_free = size;
    cb->overflow = 0;
}

/* below from fill count mods */
int cbIsFull(CircularBuffer *cb) {
    return cb->count == cb->size;
}

int cbIsEmpty(CircularBuffer *cb) {
    return cb->count == 0;
}

// returns index of next free element
int cbGetNextWrite(CircularBuffer *cb) {
    return (cb->start + cb->count) % cb->size;
}

void cbRead(CircularBuffer *cb) {
    cb->start = (cb->start + 1) % cb->size;
    -- cb->count;
}

void cbWrite(CircularBuffer *cb) {
    if (cb->count == cb->size) {
        cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
        // set overflow flag
        cb->overflow = 1;
    } else {
        ++ cb->count;
    }

    if((cb->size - cb->count) < cb->min_free)
        cb->min_free = cb->size - cb->count;
}

int cbGetNextRead(CircularBuffer *cb) {
    return cb->start;
}

// get minimum number of free elements
int cbGetMinFree(CircularBuffer *cb) {
    return cb->min_free;
}

// clear minimum number of free elements
void cbClearMinFree(CircularBuffer *cb) {
    cb->min_free = cb->size;
}

// return non-zero if there was an overflow since the last time the overflow flag was cleared
int cbIsOverflow(CircularBuffer *cb) {
    return cb->overflow;
}

// clear overflow flag
void cbClearOverflow(CircularBuffer *cb) {
    cb->overflow = 0;
}
