EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:2kwMotorController
LIBS:2kwMotorController-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title "Top Level"
Date ""
Rev "0"
Comp ""
Comment1 "Author: Oliver Douglas"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 7450 1950 1600 1250
U 5975AD12
F0 "sensors" 60
F1 "sensors.sch" 60
F2 "I_SENSE+" B L 7450 2050 60 
F3 "I_SENSE-" B L 7450 2150 60 
F4 "I_SENSOR_OUT" O R 9050 2050 60 
F5 "VBAT_CTRL" O R 9050 2250 60 
F6 "VBAT_MOTOR" O R 9050 2350 60 
F7 "VBUS_MOTOR" O R 9050 2450 60 
F8 "VBUS" I L 7450 2350 60 
$EndSheet
$Sheet
S 3400 4050 950  1250
U 5975B0C1
F0 "power" 60
F1 "power.sch" 60
$EndSheet
$Sheet
S 3350 1950 1600 1250
U 59759ACE
F0 "mcu" 60
F1 "mcu.sch" 60
F2 "PWM1_MCU" O R 4950 2050 60 
F3 "PWM2_MCU" O R 4950 2150 60 
F4 "PWM3_MCU" O R 4950 2250 60 
F5 "PWM4_MCU" O R 4950 2350 60 
F6 "VBUS_EN_MCU" O R 4950 2500 60 
F7 "VBUS_DIS_MCU" O R 4950 2600 60 
$EndSheet
Wire Wire Line
	4950 2050 5450 2050
Wire Wire Line
	5450 2150 4950 2150
Wire Wire Line
	4950 2250 5450 2250
Wire Wire Line
	5450 2350 4950 2350
Wire Wire Line
	4950 2500 5450 2500
Wire Wire Line
	5450 2600 4950 2600
$Sheet
S 5450 1950 1600 1250
U 5975734C
F0 "h-bridge" 60
F1 "h-bridge.sch" 60
F2 "PWM1_MCU" I L 5450 2050 60 
F3 "PWM2_MCU" I L 5450 2150 60 
F4 "PWM3_MCU" I L 5450 2250 60 
F5 "PWM4_MCU" I L 5450 2350 60 
F6 "VBUS_EN_MCU" I L 5450 2500 60 
F7 "VBUS_DIS_MCU" I L 5450 2600 60 
F8 "I_SENSE+" B R 7050 2050 60 
F9 "I_SENSE-" B R 7050 2150 60 
F10 "VBUS" O R 7050 2350 60 
$EndSheet
Wire Wire Line
	7050 2050 7450 2050
Wire Wire Line
	7050 2150 7450 2150
Wire Wire Line
	7050 2350 7450 2350
$EndSCHEMATC
