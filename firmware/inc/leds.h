/* leds.h file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */

#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include "stm8s.h"

#define LEDS_COUNT 2

typedef enum {
	LED_OFF 		= 0,
	LED_ON			= 1,
	LED_INVERT	= 2
} LED_ON_OFF;

void ledsOnOff(uint8_t ledNo, LED_ON_OFF on);
uint8_t getLedStatus(uint8_t ledNo);

#endif