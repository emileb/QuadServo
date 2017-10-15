//#include "utils.h"
#include "fifo.h"



void fifo_reset(fifo_t *f)
{
	f->u8RXBuffer_pos_w = 0;
	f->u8RXBuffer_pos_r = 0;
}

/****************************************************************************
 *
 * NAME: fifo_bytes
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * Number of bytes in fifo
 *
 ****************************************************************************/
unsigned int fifo_bytes(fifo_t *f)
{
	unsigned int buffer_size;

	if (f->u8RXBuffer_pos_w == f->u8RXBuffer_pos_r)
	{
		buffer_size = 0;
	}
	else if (f->u8RXBuffer_pos_w > f->u8RXBuffer_pos_r)
	{
		buffer_size = f->u8RXBuffer_pos_w - f->u8RXBuffer_pos_r;
	}
	else
	{
		buffer_size = FIFO_BUFFER_SIZE - (f->u8RXBuffer_pos_r - f->u8RXBuffer_pos_w);
	}

	//if (f->debug) vUtils_DisplayMsg("Fifo buffer_size = ",buffer_size);
	return buffer_size;
}

/****************************************************************************
 *
 * NAME: fifo_get_byte
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
unsigned char fifo_get_byte(fifo_t *f)
{
	unsigned char b = 0;
	if (fifo_bytes(f))
	{
		b =  f->pRXBuffer[f->u8RXBuffer_pos_r++];
		if (f->u8RXBuffer_pos_r == FIFO_BUFFER_SIZE) f->u8RXBuffer_pos_r = 0;
	}
	else
	{
		// if (f->debug)  vUtils_Debug("Fifo ERROR, tried to read from EMPTY fifo");
	}
	return b;
}

/****************************************************************************
 *
 * NAME: fifo_peek_byte
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
unsigned char fifo_peek_byte(fifo_t *f)
{
	unsigned char b = 0;
	if (fifo_bytes(f))
	{
		b =  f->pRXBuffer[f->u8RXBuffer_pos_r];
	}
	else
	{
		// if (f->debug)  vUtils_Debug("Fifo ERROR, tried to read from EMPTY fifo");
	}
	return b;

}
/****************************************************************************
 *
 * NAME: fifo_full
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
int fifo_full(fifo_t *f)
{
	if (fifo_bytes(f) == FIFO_BUFFER_SIZE)
		return 1;
	else
		return 0;

}
/****************************************************************************
 *
 * NAME: fifo_put_byte
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void fifo_put_byte(fifo_t *f,unsigned char b)
{

	if (!fifo_full(f))
	{
		//if (f->debug) vUtils_DisplayMsg("Fifo put byte  ",b);
		f->pRXBuffer[f->u8RXBuffer_pos_w++] = b;
		if (f->u8RXBuffer_pos_w == FIFO_BUFFER_SIZE) f->u8RXBuffer_pos_w = 0;
	}
	else
	{
		//if (f->debug) vUtils_Debug("Fifo ERROR, tried to write to FULL fifo");
	}
}


