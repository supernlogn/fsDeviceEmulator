/*
 * user_s25fl.h
 *
 *  Created on: 28 Ιουν 2017
 *      Author: ath.io
 */

#ifndef USER_S25FL_H_
#define USER_S25FL_H_

#include <s25fl_router.h>

#define S25FL_HSPI		hspi4

#define S25FL_0_PORT	GPIOB
#define S25FL_0_PIN		GPIO_PIN_14

#define S25FL_1_PORT	GPIOB
#define S25FL_1_PIN 	GPIO_PIN_15

extern s25fl_io_t s25fl_ios[2];
extern pqueue_t s25fl_queue;
extern s25fl_router_t *s25fl_router;

void user_s25fl_driver_init();
GPIO_TypeDef* user_get_s25fl_port(dev_id_t dev);
uint16_t user_get_s25fl_pin(dev_id_t dev);

#endif /* USER_S25FL_H_ */
