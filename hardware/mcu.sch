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
Sheet 4 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 "Author: Oliver Douglas"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 10400 2650 0    60   Output ~ 0
PWM1
Text HLabel 10400 2950 0    60   Output ~ 0
PWM2
Wire Wire Line
	9700 1350 9800 1350
Wire Wire Line
	9800 1350 9800 1850
Wire Wire Line
	9800 1850 9700 1850
Wire Wire Line
	9700 1750 9900 1750
Connection ~ 9800 1750
Wire Wire Line
	9700 1650 9800 1650
Connection ~ 9800 1650
Wire Wire Line
	9700 1550 9800 1550
Connection ~ 9800 1550
Wire Wire Line
	9700 1450 9800 1450
Connection ~ 9800 1450
$Comp
L GND #PWR07
U 1 1 5983EA5A
P 9900 1800
F 0 "#PWR07" H 9900 1550 50  0001 C CNN
F 1 "GND" H 9900 1650 50  0000 C CNN
F 2 "" H 9900 1800 50  0000 C CNN
F 3 "" H 9900 1800 50  0000 C CNN
	1    9900 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 1750 9900 1800
Wire Wire Line
	9700 2000 9800 2000
Wire Wire Line
	9800 2000 9800 2050
$Comp
L GNDA #PWR08
U 1 1 5983EB4E
P 9800 2050
F 0 "#PWR08" H 9800 1800 50  0001 C CNN
F 1 "GNDA" H 9800 1900 50  0000 C CNN
F 2 "" H 9800 2050 50  0000 C CNN
F 3 "" H 9800 2050 50  0000 C CNN
	1    9800 2050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR09
U 1 1 5983EC7E
P 8400 1300
F 0 "#PWR09" H 8400 1150 50  0001 C CNN
F 1 "+3.3V" H 8400 1440 50  0000 C CNN
F 2 "" H 8400 1300 50  0000 C CNN
F 3 "" H 8400 1300 50  0000 C CNN
	1    8400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1300 8400 2800
Wire Wire Line
	8400 2650 8500 2650
Wire Wire Line
	8500 2550 8400 2550
Connection ~ 8400 2550
Wire Wire Line
	8400 2450 8500 2450
Connection ~ 8400 2450
Wire Wire Line
	8500 2350 8400 2350
Connection ~ 8400 2350
Wire Wire Line
	8500 2250 8400 2250
Connection ~ 8400 2250
Wire Wire Line
	8500 2150 8400 2150
Connection ~ 8400 2150
Wire Wire Line
	8400 2050 8500 2050
Connection ~ 8400 2050
Wire Wire Line
	8500 1950 8400 1950
Connection ~ 8400 1950
Wire Wire Line
	8400 1850 8500 1850
Connection ~ 8400 1850
Wire Wire Line
	8500 1350 8400 1350
Connection ~ 8400 1350
Wire Wire Line
	8400 1450 8500 1450
Connection ~ 8400 1450
Wire Wire Line
	8500 1550 8400 1550
Connection ~ 8400 1550
Wire Wire Line
	8400 1650 8500 1650
Connection ~ 8400 1650
Wire Wire Line
	8500 1750 8400 1750
Connection ~ 8400 1750
Wire Wire Line
	8400 2800 8500 2800
Connection ~ 8400 2650
Text Notes 3250 5650 2    197  ~ 0
CAN
Text Notes 5700 5650 2    197  ~ 0
ETHERNET
Text Notes 1500 5650 2    197  ~ 0
UART
Text Notes 2550 6300 2    197  ~ 0
DEBUG BUS
Text Notes 6000 6350 2    197  ~ 0
STATUS LEDs
$Comp
L TM4C1294KCPDT U?
U 1 1 59940BF1
P 2450 2750
F 0 "U?" H 2400 1100 59  0000 C CNN
F 1 "TM4C1294KCPDT" H 2450 4500 59  0000 C CNN
F 2 "" H 2400 1500 59  0000 C CNN
F 3 "" H 2400 1500 59  0000 C CNN
	1    2450 2750
	1    0    0    -1  
$EndComp
$Comp
L TM4C1294KCPDT U?
U 2 1 59940C7A
P 5900 2250
F 0 "U?" H 5900 900 59  0000 C CNN
F 1 "TM4C1294KCPDT" H 5900 3450 59  0000 C CNN
F 2 "" H 5850 1000 59  0000 C CNN
F 3 "" H 5850 1000 59  0000 C CNN
	2    5900 2250
	1    0    0    -1  
$EndComp
$Comp
L TM4C1294KCPDT U?
U 3 1 59940CC3
P 9000 4900
F 0 "U?" H 9050 4400 59  0000 C CNN
F 1 "TM4C1294KCPDT" H 9050 5350 59  0000 C CNN
F 2 "" H 8950 3650 59  0000 C CNN
F 3 "" H 8950 3650 59  0000 C CNN
	3    9000 4900
	1    0    0    -1  
$EndComp
$Comp
L TM4C1294KCPDT U?
U 4 1 59940D20
P 9100 2300
F 0 "U?" H 9100 1100 59  0000 C CNN
F 1 "TM4C1294KCPDT" H 9100 3400 59  0000 C CNN
F 2 "" H 9050 1050 59  0000 C CNN
F 3 "" H 9050 1050 59  0000 C CNN
	4    9100 2300
	1    0    0    -1  
$EndComp
$EndSCHEMATC