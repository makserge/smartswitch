/* modbus_serial.c file
 * 
 * Copyright (c) 2016-2018 Evgeny Sobolev
 * Contact:
 *	e-mail: evgeny@vrnnet.ru
 *  skype:  evgenysbl
 *  tel.:   +7(908)1375847
 */
#include "stm8s_conf.h"
#include "modbus_serial.h"
#include "crc.h"
#include "helper.h"
#include "eeprom.h"
#include "vtimer.h"

const uint16_t gRS485_RTU_TIMEOUT[] = {
	8000,  // RS485_BAUD_4800
	4000,  // RS485_BAUD_9600
	2000,  // RS485_BAUD_19200
	1000,  // RS485_BAUD_38400
	1000,  // RS485_BAUD_57600
	1000   // RS485_BAUD_115200
};

const uint8_t HEX[] = "0123456789ABCDEF";

typedef enum {
	RTU_STATE_IDLE = 0,
	RTU_STATE_RECEIVE_DATA,
	RTU_STATE_LONG_FRAME,
	RTU_STATE_PARITY_ERROR,
	RTU_STATE_FRAME_CHECK,
	RTU_STATE_FRAME_RECEIVED,
	RTU_STATE_TRANSMIT_PAUSE,
	RTU_STATE_TRANSMIT_DATA,
	RTU_STATE_TRANSMIT_CRC_HIGH,
	RTU_STATE_TRANSMIT_CRC_LOW
} RTU_STATE;

typedef struct {
	RTU_STATE			state;
	uint8_t 			byteIndex;
	uint8_t				txByteCount;
	uint16_t			crc;
} MODBUS_RTU_STATE_MASHINE, *pMODBUS_RTU_STATE_MASHINE;

typedef struct {
	RS485_BAUD		baud;
	MODBUS_RTU_STATE_MASHINE 		rtu;
	uint8_t buf[RS485_FRAME_SIZE + 1];
} RS485_STATE, *pRS485_STATE;

void startModbusRtuTimer(void);

static const MODBUS_CONFIG gModbusDefaultConfig = {
	1,
	RS485_BAUD_9600,
	RS485_NO_PARITY		
};

MODBUS_CONFIG gModbusConfig;
static RS485_STATE gRS485State;

RS485_ERROR rs485_init(pMODBUS_CONFIG pCfg) {
	static uint8_t first = 1;
  uint16_t div;
  uint8_t cr1, cr3;
  if (first) {
		// Enable clocks for UART1 and Timer4
		CLK->PCKENR1 |= (CLK_PCKENR1_UART1);	
		// Disable TX/RX and it's interrupts
		UART1->CR2 = 0;
		memset(&gRS485State, 0, sizeof(gRS485State));		
	}	else {
		UART1->CR2 &= ~(UART1_CR2_TCIEN);
	}
	switch (pCfg->baud) {
		case RS485_BAUD_4800:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 2400)/4800);
			break;
    case RS485_BAUD_9600:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 4800)/9600);
			break;
    case RS485_BAUD_19200:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 9600)/19200);
			break;
    case RS485_BAUD_38400:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 19200)/38400);
			break;
    case RS485_BAUD_57600:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 38400)/57600);
			break;
    case RS485_BAUD_115200:
			div = (uint16_t)(((uint32_t)MCK_FREQ + 57600)/115200);
			break;    
		default:
			return RS485_ERROR_WRONG_BAUD;
  }
	gRS485State.baud = pCfg->baud;	
	switch (pCfg->parity) {
		case RS485_NO_PARITY:
			cr1 = 0;
			cr3 = (2 << 4);
			break;
		case RS485_EVEN:
			cr1 = (1 << 4) | (1 << 2) | (0 << 1);
			cr3 = 0;
			break;
		case RS485_ODD:
			cr1 = (1 << 4) | (1 << 2) | (1 << 1);
			cr3 = 0;
			break;
		default:
			return RS485_ERROR_WRONG_PARITY;
  }
	UART1->CR1 = cr1;
	UART1->CR3 = cr3;
  UART1->BRR2 = ((div & 0xF000) >> 8 ) | (div & 0x0F);
  UART1->BRR1 =  div >> 4;
	if (first) {
		first = 0;
		UART1->CR2 = (UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_TCIEN | UART1_CR2_RIEN);
	} else {
		UART1->CR2 |= (UART1_CR2_TCIEN);
	}
	return RS485_NO_ERROR;
}  

void modbusInit(void) {
	loadSerialConfig();
	rs485_init(&gModbusConfig);
}

void saveSerialConfig(void) {
	uint16_t crc;
	eepromWrite((uint8_t*)EEPROM_ADDR_MODBUS_ID, gModbusConfig.id);
	eepromWrite((uint8_t*)EEPROM_ADDR_MODBUS_BAUD, gModbusConfig.baud);
	eepromWrite((uint8_t*)EEPROM_ADDR_MODBUS_PARITY, gModbusConfig.parity);
  crc = rtuCRC((uint8_t*)EEPROM_ADDR_MODBUS_ID, 4);
	eepromWrite((uint8_t*)EEPROM_ADDR_MODBUS_CRCL, (crc & 0xFF));
	eepromWrite((uint8_t*)EEPROM_ADDR_MODBUS_CRCH, (crc >> 8));	
}

void saveDefaultSerialConfig(void) {
	gModbusConfig = gModbusDefaultConfig;
	saveSerialConfig();	
}

void loadSerialConfig(void) {
  uint16_t crc = rtuCRC((uint8_t*)EEPROM_ADDR_MODBUS_ID, 4);
	uint16_t rdCRC = LE16(EEPROM_ADDR_MODBUS_CRCL);
	if (crc == rdCRC) {
		gModbusConfig.id = *(uint8_t*) EEPROM_ADDR_MODBUS_ID;
		gModbusConfig.baud = *(pRS485_BAUD) EEPROM_ADDR_MODBUS_BAUD;
		gModbusConfig.parity = *(pRS485_PARITY)	EEPROM_ADDR_MODBUS_PARITY;
	} else {
		saveDefaultSerialConfig();
	}
}

static uint8_t symToHex(const char sym) {
	uint8_t val = 0xFF;
	if ((sym >= '0') && (sym <= '9')) {
		val = sym - '0';
	} else if ((sym >= 'A') && (sym <= 'F')) {
		val = sym - ('A' - 0x0A);
	} else if ((sym >= 'a') && (sym <= 'f')) {
		val = sym - ('a' - 0x0A);
	}
	return val;
}

void startModbusRtuTimer(void) {
	vTimerStop(2);
	vTimerStart(2, gRS485_RTU_TIMEOUT[gRS485State.baud]);		 
}

static void modbusDisableRx(void) {
	UART1->CR2 &= ~(UART1_CR2_RIEN | UART1_CR2_REN);
}

static void modbusEnableRx(void) {
	uint8_t temp = UART1->DR;
	UART1->SR &= ~UART1_SR_RXNE;
	UART1->CR2 |= (UART1_CR2_RIEN | UART1_CR2_REN);
	temp = temp;
}

INTERRUPT_HANDLER(UART1_Rx_IRQHandler, 20) {	 
	pRS485_STATE st = &gRS485State;				
	uint8_t status = UART1->SR;		
	if (status & UART1_SR_RXNE) {
		uint8_t data = UART1->DR;					
		RTU_STATE state = st->rtu.state;				
	if ((state == RTU_STATE_IDLE) ||
				(state == RTU_STATE_RECEIVE_DATA)) {
			startModbusRtuTimer();
		}
		if (status & (UART1_SR_PE | UART1_SR_OR)) {
			// Parity error
			st->rtu.state = RTU_STATE_PARITY_ERROR;
		} else if (state == RTU_STATE_IDLE) {
			// First byte received
			st->rtu.byteIndex = 1;
			st->rtu.state = RTU_STATE_RECEIVE_DATA;							
			st->buf[0] = data;
		} else if (state == RTU_STATE_RECEIVE_DATA) {
			st->buf[st->rtu.byteIndex++] = data;
			if (st->rtu.byteIndex > RS485_FRAME_SIZE) {
				// Frame too big for us
				st->rtu.state = RTU_STATE_LONG_FRAME;
			}
		}
	}
}

void vTimer2Irq(void) {	
	pRS485_STATE st = &gRS485State;
	RTU_STATE state = st->rtu.state;
	if (state == RTU_STATE_RECEIVE_DATA) {
		// Possible RTU frame received
		uint8_t len = st->rtu.byteIndex;
		st->rtu.state = RTU_STATE_FRAME_CHECK;
		if (len >= 3) {							
			// Length is 3 bytes or more
			uint8_t dataLen = len - 2;
			uint16_t crc_calc = rtuCRC(st->buf, dataLen);
			uint16_t crc_recv = LE16(st->buf + dataLen);
			if (crc_recv == crc_calc) {
				// Frame successfully received
				st->rtu.state = RTU_STATE_FRAME_RECEIVED;
				// Process frame
				if (modbusRx(st->buf, dataLen) == MODBUS_NO_FUTURE_PROCESSING_REQUIRED) {
					// Frame not need to be processed
					if (st->rtu.state == RTU_STATE_FRAME_RECEIVED) {
						// Switch to idle, if state not changed
						uint8_t temp = UART1->DR;
						temp = temp;													
						st->rtu.state = RTU_STATE_IDLE;
					}
				}
			} else {
				// CRC error
				st->rtu.state = RTU_STATE_IDLE;
			}														
		} else {
			// Frame length less then 3 bytes
			st->rtu.state = RTU_STATE_IDLE;
		}
	} else if ((state == RTU_STATE_LONG_FRAME) || 
					(state == RTU_STATE_PARITY_ERROR)) {
		// End of long frame, or parity error - switch to idle
		st->rtu.state = RTU_STATE_IDLE;
	} else if (state == RTU_STATE_TRANSMIT_PAUSE) {
		// Transmit first byte
		GPIOD->ODR |= (1 << 4); //TX_EN - PD4 (Output) 
		st->rtu.state = RTU_STATE_TRANSMIT_DATA;
		UART1->DR = st->buf[st->rtu.byteIndex++];
	}
}

INTERRUPT_HANDLER(UART1_Tx_IRQHandler, 19) {
	pRS485_STATE st = &gRS485State;	
	if (UART1->SR & UART1_SR_TC) {
		switch (st->rtu.state) {
			case RTU_STATE_TRANSMIT_DATA:
				if (st->rtu.byteIndex == st->rtu.txByteCount) {
					st->rtu.state = RTU_STATE_TRANSMIT_CRC_LOW;
					UART1->DR = (st->rtu.crc);
				} else {
					UART1->DR = st->buf[st->rtu.byteIndex++];
				}
				break;
			case RTU_STATE_TRANSMIT_CRC_LOW:
				st->rtu.state = RTU_STATE_TRANSMIT_CRC_HIGH;
				UART1->DR = (st->rtu.crc >> 8);
				break;
			case RTU_STATE_TRANSMIT_CRC_HIGH:
				st->rtu.state = RTU_STATE_IDLE;
				GPIOD->ODR &= ~(1 << 4); 	//TX_EN - PD4 (Output)
				UART1->SR &= ~UART1_SR_TC;
				// Enable receive
				modbusEnableRx();
				break;
			default:
				// Switch to RX only - Idle state
				st->rtu.state = RTU_STATE_IDLE;
				modbusEnableRx();
				GPIOD->ODR &= ~(1 << 4); 	//TX_EN - PD4 (Output) 
				UART1->SR &= ~UART1_SR_TC;	
		}
	}
}

void modbusTx(uint8_t *data, const uint8_t size) {
	RTU_STATE state = 0;
	pRS485_STATE st = &gRS485State;
	if (size == 0) return;
	modbusDisableRx();
	state = st->rtu.state;
	if (state != RTU_STATE_FRAME_RECEIVED) return;
	st->rtu.state = RTU_STATE_TRANSMIT_PAUSE;
	st->rtu.crc = rtuCRC(data, size);
	st->rtu.byteIndex = 0;
	st->rtu.txByteCount = size;
	if (st->buf != data) memcpy(st->buf, data, size);
	startModbusRtuTimer();
}
