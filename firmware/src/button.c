/* button.c file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */
#include "button.h"
#include "leds.h"
#include "vtimer.h"

#define BUTTON_FILTER_TIME_IN_10MS  (4)

typedef struct {	
	uint8_t   filteredState;
	uint8_t   changed;
	uint8_t		state;
	uint8_t 	counter;
} BUTTON_STATE, *pBUTTON_STATE;

uint8_t getButtonInputValue(uint8_t buttonNo);

static BUTTON_STATE gButtonsState[BUTTONS_COUNT];

void buttonsInit(void) {
	uint8_t butNo = 0;
	for (butNo = 0; butNo < BUTTONS_COUNT; butNo++) {
		pBUTTON_STATE bs = &gButtonsState[butNo];
		bs->state =  getButtonInputValue(butNo);
		bs->filteredState = bs->state;			
		bs->counter = bs->changed = 0;
	}
	vTimerStart(1, 50000);
}

uint8_t getButtonState(uint8_t buttonNo) {
	uint8_t inputVal = 0;
  if (buttonNo < BUTTONS_COUNT) {
		inputVal = gButtonsState[buttonNo].filteredState;
	}	
	return inputVal;
}

uint8_t getButtonInputValue(uint8_t buttonNo) {
	uint8_t inputVal = 0;
	switch (buttonNo) {
		case 0:	
			inputVal = ( ~GPIOC->IDR >> 3 ) & 0x01;
			break;
		case 1:
			inputVal = ( ~GPIOC->IDR >> 4 ) & 0x01;
			break;
	}
	return inputVal;
}

void vTimer1Irq(void) {
	uint8_t butNo = 0;
	uint8_t ledStatus = 0;
	for (butNo = 0; butNo < BUTTONS_COUNT; butNo++) {
		pBUTTON_STATE bs = &gButtonsState[butNo];
		uint8_t curState = getButtonInputValue(butNo);
		if (bs->state != curState) {
			if (bs->counter < BUTTON_FILTER_TIME_IN_10MS) {
				bs->counter++;
			} else {
				bs->filteredState = curState;						
				bs->changed |= (curState) ? 0x01 : 0x02;
				bs->state = curState;
				if (curState == 1) {
					ledStatus = getLedStatus(butNo);
					if (ledStatus == 1) {
						ledsOnOff(butNo, LED_OFF);
					} else {
						ledsOnOff(butNo, LED_ON);
					}
				}	
			}
		} else {
			bs->counter = 0;
		}
	}
	vTimerStart(1, 10000);
}
