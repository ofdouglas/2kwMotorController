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

// This file was automatically generated on 10/3/2017 at 10:57:39 AM
// by TI PinMux version 4.0.1496 
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
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    //
    // Configure the GPIO Pin Mux for PF0
	// for M0PWM0
    //
	MAP_GPIOPinConfigure(GPIO_PF0_M0PWM0);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG0
	// for M0PWM4
    //
	MAP_GPIOPinConfigure(GPIO_PG0_M0PWM4);
	MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG1
	// for M0PWM5
    //
	MAP_GPIOPinConfigure(GPIO_PG1_M0PWM5);
	MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PF2
	// for M0PWM2
    //
	MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PF3
	// for M0PWM3
    //
	MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK1
	// for AIN17
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PD0
	// for AIN15
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PD3
	// for AIN12
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK3
	// for AIN19
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD2
	// for AIN13
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PB0
	// for CAN1RX
    //
	MAP_GPIOPinConfigure(GPIO_PB0_CAN1RX);
	MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PB1
	// for CAN1TX
    //
	MAP_GPIOPinConfigure(GPIO_PB1_CAN1TX);
	MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN0
	// for GPIO_PN0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PN1
	// for GPIO_PN1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN2
	// for GPIO_PN2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PN3
	// for GPIO_PN3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD4
	// for GPIO_PD4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PD5
	// for GPIO_PD5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PD6
	// for GPIO_PD6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
	
    //
    // Configure the GPIO Pin Mux for PD7
	// for GPIO_PD7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PP2
	// for GPIO_PP2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PP3
	// for GPIO_PP3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PP4
	// for GPIO_PP4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PP5
	// for GPIO_PP5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PA0
	// for GPIO_PA0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA3
	// for GPIO_PA3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PM0
	// for GPIO_PM0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0);
	MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PM1
	// for GPIO_PM1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PM4
	// for GPIO_PM4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_4);
	MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PB2
	// for T5CCP0
    //
	MAP_GPIOPinConfigure(GPIO_PB2_T5CCP0);
	MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PJ0
	// for U3RX
    //
	MAP_GPIOPinConfigure(GPIO_PJ0_U3RX);
	MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PJ1
	// for U3TX
    //
	MAP_GPIOPinConfigure(GPIO_PJ1_U3TX);
	MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL2
	// for PHB0
    //
	MAP_GPIOPinConfigure(GPIO_PL2_PHB0);
	MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PL3
	// for IDX0
    //
	MAP_GPIOPinConfigure(GPIO_PL3_IDX0);
	MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PL1
	// for PHA0
    //
	MAP_GPIOPinConfigure(GPIO_PL1_PHA0);
	MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

