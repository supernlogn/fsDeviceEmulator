/*
 * user_s25fl.c
 *
 *  Created on: 28 Ιουν 2017
 *      Author: ath.io
 */
#include "user_s25fl.h"
#include <ffconf.h>

s25fl_io_t s25fl_ios[_VOLUMES];

pqueue_t s25fl_queue = NULL;
s25fl_io_t* s25fl_io = NULL;
s25fl_router_t *s25fl_router;

queue_t user_queue;
BYTE user_queue_buffer[64 * sizeof(s25fl_msg_t)];
s25fl_router_t user_s25fl_router;


void user_s25fl_driver_init()
{
	/* initialize a queue */
	s25fl_queue = &user_queue;
	queue_init(s25fl_queue, sizeof(s25fl_msg_t), 100, s25fl_msg_free_func, user_queue_buffer);

	/* initialize the s25fl_router */
	s25fl_router = &user_s25fl_router;
	s25fl_router_link_create(&user_s25fl_router, s25fl_queue);
}

GPIO_TypeDef* user_get_s25fl_port(dev_id_t dev)
{
	GPIO_TypeDef* res = NULL;
	switch(dev)
	{
		case dev_25fl_0:
			res = S25FL_0_PORT;
			break;
		case dev_25fl_1:
			res = S25FL_1_PORT;
			break;
		default:
			res = NULL;
			break;
	}

	return res;
}

uint16_t user_get_s25fl_pin(dev_id_t dev)
{
	uint16_t res = S25FL_0_PIN;
	switch(dev)
	{
		case dev_25fl_0:
			res = S25FL_0_PIN;
			break;
		case dev_25fl_1:
			res = S25FL_1_PIN;
			break;
		default:
			res = 0xFF; // invalid pin
			break;
	}

	return res;
}
