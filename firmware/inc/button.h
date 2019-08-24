/* button.h file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */

#ifndef __BUTTON_H_INCLUDED
#define __BUTTON_H_INCLUDED

#include "stm8s.h"

#define BUTTONS_COUNT 2

uint8_t getButtonState(uint8_t buttonNo);
void buttonsInit(void);

#endif // __BUTTON_H_INCLUDED