# SmartSwitch
Modbus RTU wall switch

2 way Modbus switch with local input control based on STM8S003F3PU

Hardware is 2 way ModbusRTU Relay from China: 2 active high level inputs and 2 LED / relay outputs.

Default settings:

Slave ID: 1
Serial Port: 

Baud rate: 9600
Data bits: 8
Parity: None
Stop Bits: 1

Registers definition:

0: Slave ID
1: Baud rate (0 - 4800, 1 - 9600, 2 - 19200, 3 - 38400, 4 - 57600, 5 - 115200)
2: Parity (0 - No Parity, 1 - Even, 2 - Odd)
3: Save params (1 - Save)
4: Led Status (0 - Led1, Led2 off, 1 - Led 1 on, 2 - Led 2 on, 3 - Led1, Led2 on)

ST Visual Develop project
