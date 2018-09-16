/* 
 * File:   circbuffer.h
 * Author: ejoseph
 *
 * Created on April 30, 2018, 11:14 PM
 */

#ifndef CIRCBUFFER_H
#define CIRCBUFFER_H

#include <stdint.h>
#include <stddef.h>

#define ERROR_BUFFER_FULL -1
#define ERROR_BUFFER_EMPTY -1
#define ACTION_BUFFER_OK 0

typedef struct {
	void *buffer;
	size_t element_size;
	uint32_t length;
	uint32_t head;
	uint32_t tail;
	uint32_t residual;
	bool buferring;
} circ_buffer_t;

void CIRC_BUFFER_Init(circ_buffer_t *cbuffer_t, void *buffer,
		size_t element_size, uint32_t length);

void CIRC_BUFFER_InitRecidual(circ_buffer_t *cbuffer_t, void *buffer,
		size_t element_size, uint32_t length, uint32_t residual);

int32_t CIRC_BUFFER_push(circ_buffer_t *cbuffer_t, void *data);

int32_t CIRC_BUFFER_pop(circ_buffer_t *cbuffer_t, void *data);

bool CIRC_BUFFER_hasSpace(circ_buffer_t *cbuffer_t);

uint32_t CIRC_BUFFER_elementsInBuffer(circ_buffer_t *cbuffer_t);

#endif /* CIRCBUFFER_H */
