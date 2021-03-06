//*****************************************************************************
//
// Configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 20.04.2018 at 20:17:00
// by TI PinMux version 4.0.1511
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the customer specific usage.
//!
//! \return None.
//
//*****************************************************************************
void
PinoutSet(void)
{
    //
    // Enable Peripheral Clocks 
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Unlock the Port Pin and Set the Commit Bit
    //
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;

    //
    // Configure the GPIO Pin Mux for PD7
    // for WT5CCP1
    //
    MAP_GPIOPinConfigure(GPIO_PD7_WT5CCP1);
    MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PD6
    // for WT5CCP0
    //
    MAP_GPIOPinConfigure(GPIO_PD6_WT5CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PC5
    // for WT0CCP1
    //
    MAP_GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    MAP_GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PD5
    // for USB0DP
    //
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PD4
    // for USB0DM
    //
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PD3
    // for AIN4
    //
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD2
    // for AIN5
    //
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PE4
    // for GPIO_PE4
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PE5
    // for GPIO_PE5
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PE2
    // for GPIO_PE2
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PE3
    // for GPIO_PE3
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PE0
    // for GPIO_PE0
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PE1
    // for GPIO_PE1
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PA5
    // for GPIO_PA5
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PA2
    // for GPIO_PA2
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PA3
    // for GPIO_PA3
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PA4
    // for GPIO_PA4
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);

    //
    // Unlock the Port Pin and Set the Commit Bit
    //
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR)   |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0x0;

    //
    // Configure the GPIO Pin Mux for PF0
    // for GPIO_PF0
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PF4
    // for GPIO_PF4
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PF1
    // for GPIO_PF1
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);

    //
    // Configure the GPIO Pin Mux for PF3
    // for GPIO_PF3
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);

    //
    // Configure the GPIO Pin Mux for PA0
    // for GPIO_PA0
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_0);
    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PC6
    // for GPIO_PC6
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PC7
    // for GPIO_PC7
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PA6
    // for GPIO_PA6
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PA7
    // for GPIO_PA7
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x0);

    //
    // Configure the GPIO Pin Mux for PB6
    // for SSI2RX
    //
    MAP_GPIOPinConfigure(GPIO_PB6_SSI2RX);
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PB7
    // for SSI2TX
    //
    MAP_GPIOPinConfigure(GPIO_PB7_SSI2TX);
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PB4
    // for SSI2CLK
    //
    MAP_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PF2
    // for T1CCP0
    //
    MAP_GPIOPinConfigure(GPIO_PF2_T1CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PB5
    // for T1CCP1
    //
    MAP_GPIOPinConfigure(GPIO_PB5_T1CCP1);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PB0
    // for T2CCP0
    //
    MAP_GPIOPinConfigure(GPIO_PB0_T2CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PB1
    // for T2CCP1
    //
    MAP_GPIOPinConfigure(GPIO_PB1_T2CCP1);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
