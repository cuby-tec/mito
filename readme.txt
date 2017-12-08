
    // Configure the GPIO Pin Mux for PB4
    // for T1CCP0	X-axis
    // Configure the GPIO Pin Mux for PB5
    // for T1CCP1	Y-axis
    
    // Configure the GPIO Pin Mux for PB0
    // for T2CCP0	Z-axis
    
    // Configure the GPIO Pin Mux for PB1
    // for T2CCP1	E-axis




#define DIRECTION_PORT      GPIO_PORTE_BASE
#define DIRECTION_X         GPIO_PIN_4
#define DIRECTION_Y         GPIO_PIN_3
#define DIRECTION_Z         GPIO_PIN_2
#define DIRECTION_E         GPIO_PIN_1

//---------- PINS TM4C123G
+3.3V   +5V         PF2     GND
PB5     GND         PF3     PB2
PB0     PD0 PB6     PB3     PE0
PB1     PD1 PB7     PC4     PF0
PE4     PD2         PC5     RST
PE5     PD3         PC6     PB7 PD1
PB4     PE1         PC7     PB6 PD0
PA5     PE2         PD6     PA4
PA6     PE3         PD7     PA3
PA7     PF1         PF4     PA2

SSI pins
CLK PB4/58
RX	PB6/1
TX	PB7/4

//--------------------
29.08.17	ms_nextSector - задержка состявляет 25uS 
			от прерывания по завершению сегмента - continueBlock.
			
	volatile eTaskState state;
	state = eTaskGetState(orderlyHandling);
	
Текущее задание: loadMicrosteps@ms_init.c