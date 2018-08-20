/* 
 * File:   circbuffer.c
 * Author: Ericson Joseph
 * 
 * Created on April 30, 2018, 11:14 PM
 */

#include <string.h>
#include <stdbool.h>
#include "circbuffer.h"

static bool bufferIsFull(uint32_t next, uint32_t tail);

static bool bufferIsEmpty(uint32_t next, uint32_t tail);

void CIRC_BUFFER_Init(circ_buffer_t *cbuffer_t, void *buffer,
		size_t element_size, uint32_t length) {
	cbuffer_t->buffer = buffer;
	cbuffer_t->element_size = element_size;
	cbuffer_t->length = length;
	cbuffer_t->head = 0;
	cbuffer_t->tail = 0;
}

int32_t CIRC_BUFFER_push(circ_buffer_t *cbuffer_t, void *data) {
	uint32_t next = cbuffer_t->head + 1;
	next = next % cbuffer_t->length;
	if (bufferIsFull(next, cbuffer_t->tail)) {
		return ERROR_BUFFER_FULL;
	}
	cbuffer_t->head = next;
	void *point = cbuffer_t->buffer
			+ (cbuffer_t->head * cbuffer_t->element_size);
	memcpy(point, data, cbuffer_t->element_size);
	return ACTION_BUFFER_OK;
}

int32_t CIRC_BUFFER_pop(circ_buffer_t *cbuffer_t, void *data) {
	if (bufferIsEmpty(cbuffer_t->head, cbuffer_t->tail)) {
		return ERROR_BUFFER_EMPTY;
	}
	uint32_t next = cbuffer_t->tail + 1;
	cbuffer_t->tail = next % cbuffer_t->length;
	void *point = cbuffer_t->buffer
			+ (cbuffer_t->tail * cbuffer_t->element_size);
	memcpy(data, point, cbuffer_t->element_size);
	return ACTION_BUFFER_OK;
}

bool CIRC_BUFFER_hasSpace(circ_buffer_t *cbuffer_t) {
	uint32_t next = cbuffer_t->head + 1;
	next = next % cbuffer_t->length;
	return !bufferIsFull(next, cbuffer_t->tail);
}

bool bufferIsFull(uint32_t next, uint32_t tail) {
	return (next == tail);
}

bool bufferIsEmpty(uint32_t head, uint32_t tail) {
	return (head == tail);
}
