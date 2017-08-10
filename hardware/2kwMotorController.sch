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
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 5550 1950 900  950 
U 5975734C
F0 "h-bridge" 60
F1 "h-bridge.sch" 60
F2 "PWM1" I L 5550 2200 60 
F3 "PWM2" I L 5550 2400 60 
$EndSheet
$Sheet
S 5550 3550 900  800 
U 59759ACE
F0 "mcu" 60
F1 "mcu.sch" 60
F2 "PWM1" O L 5550 3900 60 
F3 "PWM2" O L 5550 3750 60 
$EndSheet
Wire Wire Line
	5550 2400 5450 2400
Wire Wire Line
	5450 2400 5450 3750
Wire Wire Line
	5450 3750 5550 3750
Wire Wire Line
	5550 3900 5350 3900
Wire Wire Line
	5350 3900 5350 2200
Wire Wire Line
	5350 2200 5550 2200
$Sheet
S 7650 2800 1200 1050
U 5975AD12
F0 "sensors" 60
F1 "sensors.sch" 60
F2 "ISENSE+" I L 7650 3050 60 
F3 "ISENSE-" O R 8850 3050 60 
F4 "CURRENT_SENSOR_OUT" O L 7650 3400 60 
$EndSheet
$Sheet
S 3150 2750 950  1250
U 5975B0C1
F0 "power" 60
F1 "power.sch" 60
$EndSheet
$EndSCHEMATC
