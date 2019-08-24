/* leds.c file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */
#include "leds.h"

void ledsOnOff(uint8_t ledNo, LED_ON_OFF on) {
	uint8_t ledMask = (1 << (3 - ledNo));
	switch (on) {
		case LED_OFF:
			GPIOD->ODR &= ~ledMask;
			break;
		case LED_ON:
			GPIOD->ODR |= ledMask;
			break;
		case LED_INVERT:
			GPIOD->ODR ^= ledMask;
			break;
	}
}

uint8_t getLedStatus(uint8_t ledNo) {
	uint8_t ledMask = (1 << (3 - ledNo));
	uint8_t status = ((GPIOD->ODR & ledMask) != 0);
	return status;
}
