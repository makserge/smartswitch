/* main.c file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */
 
#include "stm8s_conf.h"
#include "stm8s.h"
#include "clock.h"
#include "modbus.h"
#include "modbus_serial.h"
#include "leds.h"
#include "eeprom.h"
#include "helper.h"
#include "button.h"
#include "vtimer.h"

void gpioInit(void);

void main() {
	clockInit();
	gpioInit();
	vTimerInit();
	buttonsInit();
	//saveDefaultSerialConfig();
	modbusInit();
	enableInterrupts();
	while(1) {
  }
}

/*********************************************************
 * GPIO and Peripherals PINS
 *********************************************************/
void gpioInit(void) {
  // PA3 as output
  GPIOA->DDR = (1 << 3);
	GPIOA->ODR = (1 << 3);
	GPIOA->CR1 = (1 << 3);
	GPIOA->ODR = 0; 
  
  // PB4 & PB5 as output
  GPIOB->DDR = (1 << 4) | (1 << 5);
  GPIOB->CR1 = (1 << 4) | (1 << 5);
  GPIOB->CR2 = (1 << 4) | (1 << 5);
	GPIOB->ODR = 0;

  /*****************************/
  //  Port C
  //              PC3       PC4				PC5					PC6       PC7
  GPIOC->DDR = (0 << 3) | (0 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
  GPIOC->CR1 = (0 << 3) | (0 << 4) | (1 << 6) | (1 << 6) | (1 << 7);
  GPIOC->CR2 = (0 << 3) | (0 << 4) | (1 << 4) | (1 << 5) | (1 << 7);
  GPIOC->ODR = 0; 
  
  // Configure RX, TX, AND pin used for TX Enable
  // RX    - PD6 (Pullup without interrupt)
  // TX    - PD5 (Output)
  // TX_EN - PD4 (Output)
  GPIOD->DDR = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (0 << 6);
  GPIOD->CR1 = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
  GPIOD->CR2 = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (0 << 6);
  GPIOD->ODR = 0;
}
