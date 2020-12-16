#ifndef _CIRCULARBUFFER_H_
#define _CIRCULARBUFFER_H_

// TODO: Consider INLINE for several functions - many small, only used in one place, in frequently used code

/* Circular buffer object */
typedef struct {
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         count;    /* new  */
    int         min_free; /* minimum number of free elements in buffer - low water mark, manually cleared */
    int         overflow;         
} CircularBuffer;

void cbInit(CircularBuffer *cb, int size);

int cbIsFull(CircularBuffer *cb);

int cbIsEmpty(CircularBuffer *cb);

// returns index of next element to write
int cbGetNextWrite(CircularBuffer *cb);

// mark next element as written - on overflow, overwrite oldest data and set overflow flag
void cbWrite(CircularBuffer *cb);

// returns index of next element to read
int cbGetNextRead(CircularBuffer *cb);

// marks next element as read
void cbRead(CircularBuffer *cb);

// get minimum number of free elements
int cbGetMinFree(CircularBuffer *cb);

// clear minimum number of free elements
void cbClearMinFree(CircularBuffer *cb);

// return non-zero if there was an overflow since the last time the overflow flag was cleared
int cbIsOverflow(CircularBuffer *cb);

// clear overflow flag
void cbClearOverflow(CircularBuffer *cb);

#endif // _CIRCULARBUFFER_H_
