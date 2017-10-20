/*
 * pinconfig.c - Configure pins based on the build target specified in build.h
 *
 *  Created on: Oct 18, 2017
 *      Author: Oliver Douglas
 */

/* T5CCP0 is used for PWM fan
 * All other timers are free for our use
 */

#include "stdinclude.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"

// File index for ASSERT() macro
FILENUM(8)

void pinconfig(void)
{
#if BUILD_TARGET == FOO

#elif BUILD_TARGET == BUILD_TARGET_LAUNCHPAD

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    /* Configure log UART pins:
     *  UART0RX = PA0
     *  UART0TX = PA1
     */
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    /* Configure ADC test pins:
     *  AIN3 = PE0
     */
     MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Configure status LED pins:
     *  PN1 = LED0
     *  PN0 = LED1
     *  PF4 = LED2
     *  PF0 = LED3
     */
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);


#elif BUILD_TARGET == BUILD_TARGET_2KWMC_R0

    // Enable Peripheral Clocks
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

    /* Configure PWM Pins:
     *  M0PWM0 = PF0 (42): VBUS_DIS_MCU
     *  M0PWM2 = PF2 (44): PWM1_MCU
     *  M0PWM3 = PF3 (45): PWM2_MCU
     *  M0PWM4 = PG0 (49): PWM4_MCU
     *  M0PWM5 = PG1 (50): PWM3_MCU
     */
    MAP_GPIOPinConfigure(GPIO_PF0_M0PWM0);
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
    MAP_GPIOPinConfigure(GPIO_PG0_M0PWM4);
    MAP_GPIOPinConfigure(GPIO_PG1_M0PWM5);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);

    /* Configure ADC pins:
     *  AIN12 = PD3 ( 4): BRIDGE_TEMP
     *  AIN13 = PD2 ( 3): MOTOR_TEMP
     *  AIN15 = PD0 ( 1): VBAT_CTRL
     *  AIN17 = PK1 (19): VBUS_MOTOR
     *  AIN19 = PK3 (21): I_SENSOR_OUT
     */
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    /* Configure CAN pins:
     *  CAN1RX = PB0 (95): CAN1_RX
     *  CAN1TX = PB1 (96): CAN1_TX
     */
    //
    MAP_GPIOPinConfigure(GPIO_PB0_CAN1RX);
    MAP_GPIOPinConfigure(GPIO_PB1_CAN1TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_1);

    /* Configure Debug pins:
     *  PN0 (107): DEBUG_0
     *  PN1 (108): DEBUG_1
     *  PN2 (109): DEBUG_2
     *  PN3 (110): DEBUG_3
     *  PD4 (125): DEBUG_4
     *  PD5 (126): DEBUG_5
     *  PD6 (127): DEBUG_6
     *  PD7 (128): DEBUG_7
     */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

    // Use of PD7 requires unlock, since it can be used for NMI.
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    /* Configure LED pins:
     *  PP2 (103): LED_0
     *  PP3 (104): LED_1
     *  PP4 (105): LED_2
     *  PP5 (106): LED_3
     */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);

    /* Configure H-Bridge control pins:
     *  PA6 (40): VBUS_EN_MCU
     *  PA7 (41): VBUS_ST_MCU
     */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

    /* Configure CAN ID pins:
     *  PM0 (78): CAN_ID_0
     *  PM1 (77): CAN_ID_1
     */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_1);
    MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);

    /* Configure Factory Reset pin:
     *  PM4 (74): FRST
     */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_4);
    MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);

    /* Configure fan PWM pin:
     *  T5CCP0 = PB2 (91): PWM_FAN
     */
    MAP_GPIOPinConfigure(GPIO_PB2_T5CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);

    /* Configure log UART pins:
     *  UART3RX = PJ0 (116): MCU_UART3_RX
     *  UART3TX = PJ1 (117): MCU_UART3_TX
     */
    MAP_GPIOPinConfigure(GPIO_PJ0_U3RX);
    MAP_GPIOPinConfigure(GPIO_PJ1_U3TX);
    MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_1);

    /* Configure QEI pins:
     *  QEI0PHA = PL1 (82): QEI_PHA
     *  QEI0PHB = PL2 (83): QEI_PHB
     *  QEI0IDX = PL3 (84): QEI_IDX
     */
    MAP_GPIOPinConfigure(GPIO_PL1_PHA0);
    MAP_GPIOPinConfigure(GPIO_PL2_PHB0);
    MAP_GPIOPinConfigure(GPIO_PL3_IDX0);
    MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_3);

#endif
}

