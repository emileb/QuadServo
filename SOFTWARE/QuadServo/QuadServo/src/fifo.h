
#ifndef FIFO_H
#define FIFO_H

#define FIFO_BUFFER_SIZE 512

typedef struct
{
    unsigned char pRXBuffer[FIFO_BUFFER_SIZE];
    int u8RXBuffer_pos_w;
    int u8RXBuffer_pos_r;
    int debug;
}fifo_t;

 void fifo_reset(fifo_t *f);
 unsigned int fifo_bytes(fifo_t *f);
 unsigned char fifo_get_byte(fifo_t *f);
 unsigned char fifo_peek_byte(fifo_t *f);
 int fifo_full(fifo_t *f);
 void fifo_put_byte(fifo_t *f,unsigned char b);

#endif  // FIFO_H
