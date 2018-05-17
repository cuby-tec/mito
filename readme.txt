
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

//*******************
 LEDTask 			// LEDTaskInit
taskSectorhandler	// createTaskSectorHandler
SwitchTask 			// SwitchTaskInit
orderly_routine		// createtask_orderly
//***********
//***************** USB *********
Правильность назначения PIN
    //
    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

Проверка подключения USB устройства:
$ lsusb
	.........
	Bus 003 Device 044: ID 1cbe:0003 Luminary Micro Inc. 
	......... etc

lsusb -v | grep -E <'044'>

...	Bus 003 Device 044: ID 1cbe:0003 Luminary Micro Inc. .. 

//-------------- compiler
from:
${COM_TI_TM4C_INSTALL_DIR}/usblib
to:
/home/walery/workspace_v8/usblib/Debug/

from:
${SW_ROOT}/usblib/ccs/Debug/usblib.lib

//========= Vector table
USB0DeviceIntHandler

USBDeviceIntHandlerInternal

[page 124]====================
■ Software writes to the corresponding interrupt set-pending register bit, or to the Software Trigger
Interrupt (SWTRIG) register to make a Software-Generated Interrupt pending. See the INT bit
in the PEND0 register on page 146 or SWTRIG on page 156.

A pending interrupt remains pending until one of the following:
■ Software writes to the corresponding interrupt clear-pending register bit
– For a level-sensitive interrupt, if the interrupt signal is still asserted, the state of the interrupt
does not change. Otherwise, the state of the interrupt changes to inactive.
– For a pulse interrupt, the state of the interrupt changes to inactive, if the state was pending
or to active, if the state was active and pending.

[page 156]====================
Register 64: Software Trigger Interrupt (SWTRIG), offset 0xF00
Note:
Only privileged software can enable unprivileged access to the SWTRIG register.
Writing an interrupt number to the SWTRIG register generates a Software Generated Interrupt (SGI).
See Table 2-9 on page 104 for interrupt assignments.
When the MAINPEND bit in the Configuration and Control (CFGCTRL) register (see page 168) is
set, unprivileged software can access the SWTRIG register.
Software Trigger Interrupt (SWTRIG)
Base 0xE000.E000
Offset 0xF00
Type WO, reset 0x0000.0000
31 30 29 28 27 26 25 24

15 14 13 12 11 10 9 RO
23 22 21 20 19 18 17 16

8 7 6 5 4 3 2 1 0
INTID

Bit/Field Name Type Reset Description

31:8 reserved RO 0x0000.00
Software should not rely on the value of a reserved bit. To provide
compatibility with future products, the value of a reserved bit should be
preserved across a read-modify-write operation.

7:0 INTID WO 0x00
Interrupt ID
This field holds the interrupt ID of the required SGI. For example, a value
of 0x3 generates an interrupt on IRQ3.

page [107] ===========================
Figure 2-6. Vector Table  
Exception number IRQ number  Offset  Vector
154                 138      0x0268  IRQ131
    .                .           .   
18                  2       0x004C  
17                  1       0x0048  IRQ2
16                  0       0x0044  IRQ1
15                  -1      0x0040  IRQ0
14                  -2      0x003C  Systick
13                                  Reserved
12                                  Reserved for Debug
11                  -5      0x002C  SVCall
0x0038  PendSV
  
.               .       .   

. . .   


10  Reserved
9   
8
7
6                -10        0x0018  Usage fault
5                -11        0x0014  Bus fault
4               -12         0x0010  Memory management fault
3               -13         0x000C  Hard fault
2               -14         0x0008  NMI
1                          0x0004   Reset
                            0x0000  Initial SP value



