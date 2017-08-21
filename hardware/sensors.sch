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
Sheet 2 5
Title "sensors"
Date ""
Rev ""
Comp ""
Comment1 "Author: Oliver Douglas"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LAH-100P U3
U 1 1 5975C20B
P 2200 1150
F 0 "U3" H 2200 850 60  0000 C CNN
F 1 "LAH-100P" H 2200 1400 60  0000 C CNN
F 2 "" H 2200 1150 60  0001 C CNN
F 3 "" H 2200 1150 60  0000 C CNN
	1    2200 1150
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5975C452
P 2900 1600
F 0 "R5" V 2980 1600 50  0000 C CNN
F 1 "27.5" V 2900 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2830 1600 50  0001 C CNN
F 3 "" H 2900 1600 50  0000 C CNN
	1    2900 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5975C471
P 2900 1850
F 0 "#PWR01" H 2900 1600 50  0001 C CNN
F 1 "GND" H 2900 1700 50  0000 C CNN
F 2 "" H 2900 1850 50  0000 C CNN
F 3 "" H 2900 1850 50  0000 C CNN
	1    2900 1850
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5975C496
P 3150 1600
F 0 "C2" H 3175 1700 50  0000 L CNN
F 1 "200n" H 3175 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3188 1450 50  0001 C CNN
F 3 "" H 3150 1600 50  0000 C CNN
	1    3150 1600
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U6
U 1 1 5975C520
P 4450 1450
F 0 "U6" H 4500 1650 50  0000 C CNN
F 1 "MCP6001" H 4650 1250 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SC-70-5" H 4400 1550 50  0001 C CNN
F 3 "" H 4500 1650 50  0000 C CNN
	1    4450 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5975C76F
P 4350 1800
F 0 "#PWR02" H 4350 1550 50  0001 C CNN
F 1 "GND" H 4350 1650 50  0000 C CNN
F 2 "" H 4350 1800 50  0000 C CNN
F 3 "" H 4350 1800 50  0000 C CNN
	1    4350 1800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR03
U 1 1 5975C874
P 4350 1000
F 0 "#PWR03" H 4350 850 50  0001 C CNN
F 1 "+3V3" H 4350 1140 50  0000 C CNN
F 2 "" H 4350 1000 50  0000 C CNN
F 3 "" H 4350 1000 50  0000 C CNN
	1    4350 1000
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5975C99C
P 3850 1350
F 0 "R7" V 3930 1350 50  0000 C CNN
F 1 "10k" V 3850 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3780 1350 50  0001 C CNN
F 3 "" H 3850 1350 50  0000 C CNN
	1    3850 1350
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 5975CA28
P 3850 1150
F 0 "R6" V 3930 1150 50  0000 C CNN
F 1 "10k" V 3850 1150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3780 1150 50  0001 C CNN
F 3 "" H 3850 1150 50  0000 C CNN
	1    3850 1150
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR04
U 1 1 5975CA4E
P 3600 1100
F 0 "#PWR04" H 3600 950 50  0001 C CNN
F 1 "+3V3" H 3600 1240 50  0000 C CNN
F 2 "" H 3600 1100 50  0000 C CNN
F 3 "" H 3600 1100 50  0000 C CNN
	1    3600 1100
	1    0    0    -1  
$EndComp
Text Notes 1850 1700 0    60   ~ 0
IS_OUT <= 60mA\nBI-DIRECTIONAL
Text HLabel 1600 1050 0    60   BiDi ~ 12
I_SENSE+
Text HLabel 2800 1050 2    60   BiDi ~ 12
I_SENSE-
Text HLabel 5050 1450 2    60   Output ~ 12
I_SENSOR_OUT
Text Notes 3050 750  2    118  ~ 24
motor current sensor
Text Notes 9900 1100 2    118  ~ 24
motor temp sensor
Text Notes 4000 4450 2    118  ~ 24
motor battery voltage sensor
Text Notes 9900 2700 2    118  ~ 24
h-bridge temp sensor
Text Notes 3600 6100 2    118  ~ 24
controller battery voltage sensor
Text Notes 9700 4600 2    118  ~ 24
encoder interface
Text Notes 2400 2550 2    118  ~ 24
bus voltage sensor
$Comp
L +9V #PWR05
U 1 1 597CF4DC
P 1000 6600
F 0 "#PWR05" H 1000 6450 50  0001 C CNN
F 1 "+9V" H 1000 6740 50  0000 C CNN
F 2 "" H 1000 6600 50  0000 C CNN
F 3 "" H 1000 6600 50  0000 C CNN
	1    1000 6600
	1    0    0    -1  
$EndComp
Text Notes 1150 6350 2    60   ~ 0
9V nominal\n10V max
$Comp
L R R1
U 1 1 597CFA75
P 1000 6850
F 0 "R1" V 1080 6850 50  0000 C CNN
F 1 "200k" V 1000 6850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 930 6850 50  0001 C CNN
F 3 "" H 1000 6850 50  0000 C CNN
	1    1000 6850
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 597CFAC7
P 1000 7250
F 0 "R2" V 1080 7250 50  0000 C CNN
F 1 "100k" V 1000 7250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 930 7250 50  0001 C CNN
F 3 "" H 1000 7250 50  0000 C CNN
	1    1000 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 597CFAF9
P 1000 7450
F 0 "#PWR06" H 1000 7200 50  0001 C CNN
F 1 "GND" H 1000 7300 50  0000 C CNN
F 2 "" H 1000 7450 50  0000 C CNN
F 3 "" H 1000 7450 50  0000 C CNN
	1    1000 7450
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U2
U 1 1 597CFF4C
P 1650 6950
F 0 "U2" H 1700 7150 50  0000 C CNN
F 1 "MCP6001" H 1850 6750 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SC-70-5" H 1600 7050 50  0001 C CNN
F 3 "" H 1700 7150 50  0000 C CNN
	1    1650 6950
	1    0    0    1   
$EndComp
$Comp
L +3V3 #PWR07
U 1 1 597D01E3
P 1550 6600
F 0 "#PWR07" H 1550 6450 50  0001 C CNN
F 1 "+3V3" H 1550 6740 50  0000 C CNN
F 2 "" H 1550 6600 50  0000 C CNN
F 3 "" H 1550 6600 50  0000 C CNN
	1    1550 6600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 597D0261
P 1550 7300
F 0 "#PWR08" H 1550 7050 50  0001 C CNN
F 1 "GND" H 1550 7150 50  0000 C CNN
F 2 "" H 1550 7300 50  0000 C CNN
F 3 "" H 1550 7300 50  0000 C CNN
	1    1550 7300
	1    0    0    -1  
$EndComp
Text HLabel 2250 6950 2    60   Output ~ 12
VBAT_CTRL
Text HLabel 4450 5200 2    60   Output ~ 12
VBAT_MOTOR
Text HLabel 4250 3150 2    60   Output ~ 12
VBUS_MOTOR
$Comp
L R R4
U 1 1 597E87B7
P 2150 3150
F 0 "R4" V 2230 3150 50  0000 C CNN
F 1 "470" V 2150 3150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2080 3150 50  0001 C CNN
F 3 "" H 2150 3150 50  0000 C CNN
	1    2150 3150
	0    1    1    0   
$EndComp
$Comp
L LOC110 U4
U 1 1 597E8D03
P 2800 3200
F 0 "U4" H 2800 2950 60  0000 C CNN
F 1 "LOC110" H 2800 3450 60  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 2950 3000 60  0001 C CNN
F 3 "" H 2950 3000 60  0000 C CNN
	1    2800 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 597E8EB7
P 1500 3500
F 0 "#PWR09" H 1500 3250 50  0001 C CNN
F 1 "GND" H 1500 3350 50  0000 C CNN
F 2 "" H 1500 3500 50  0000 C CNN
F 3 "" H 1500 3500 50  0000 C CNN
	1    1500 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 597E9194
P 1200 3050
F 0 "#PWR010" H 1200 2800 50  0001 C CNN
F 1 "GND" H 1200 2900 50  0000 C CNN
F 2 "" H 1200 3050 50  0000 C CNN
F 3 "" H 1200 3050 50  0000 C CNN
	1    1200 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 597E9761
P 2450 3450
F 0 "#PWR011" H 2450 3200 50  0001 C CNN
F 1 "GND" H 2450 3300 50  0000 C CNN
F 2 "" H 2450 3450 50  0000 C CNN
F 3 "" H 2450 3450 50  0000 C CNN
	1    2450 3450
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 597E9880
P 1050 3750
F 0 "R3" V 1130 3750 50  0000 C CNN
F 1 "1.2M" V 1050 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 980 3750 50  0001 C CNN
F 3 "" H 1050 3750 50  0000 C CNN
	1    1050 3750
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 597E9DB4
P 1950 3500
F 0 "C1" H 1975 3600 50  0000 L CNN
F 1 "100p" H 1975 3400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1988 3350 50  0001 C CNN
F 3 "" H 1950 3500 50  0000 C CNN
	1    1950 3500
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U5
U 1 1 597EAA48
P 3700 3150
F 0 "U5" H 3750 3350 50  0000 C CNN
F 1 "MCP6001" H 3900 2950 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SC-70-5" H 3650 3250 50  0001 C CNN
F 3 "" H 3750 3350 50  0000 C CNN
	1    3700 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 597EB09C
P 3600 3500
F 0 "#PWR012" H 3600 3250 50  0001 C CNN
F 1 "GND" H 3600 3350 50  0000 C CNN
F 2 "" H 3600 3500 50  0000 C CNN
F 3 "" H 3600 3500 50  0000 C CNN
	1    3600 3500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 597EB11C
P 3600 2800
F 0 "#PWR013" H 3600 2650 50  0001 C CNN
F 1 "+3V3" H 3600 2940 50  0000 C CNN
F 2 "" H 3600 2800 50  0000 C CNN
F 3 "" H 3600 2800 50  0000 C CNN
	1    3600 2800
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 597EB265
P 3800 2500
F 0 "R8" V 3880 2500 50  0000 C CNN
F 1 "100k" V 3800 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3730 2500 50  0001 C CNN
F 3 "" H 3800 2500 50  0000 C CNN
	1    3800 2500
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 597EB397
P 3450 2350
F 0 "C3" H 3475 2450 50  0000 L CNN
F 1 "100p" H 3475 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3488 2200 50  0001 C CNN
F 3 "" H 3450 2350 50  0000 C CNN
	1    3450 2350
	0    1    1    0   
$EndComp
Text Notes 2000 4050 2    59   ~ 0
consider shifting VBUS\nso we can sense VBUS < 0
$Comp
L LM358 U1
U 1 1 598553D5
P 1600 3150
F 0 "U1" H 1550 3350 50  0000 L CNN
F 1 "LM358" H 1550 2900 50  0000 L CNN
F 2 "SMD_Packages:SOIC-8-N" H 1600 3150 50  0001 C CNN
F 3 "" H 1600 3150 50  0000 C CNN
	1    1600 3150
	1    0    0    -1  
$EndComp
Text Notes 9700 5250 2    59   ~ 0
signswise: NPN open collector\nsome others: differential
$Comp
L +12V_ISO #PWR014
U 1 1 59A951A8
P 1500 2800
F 0 "#PWR014" H 1500 2650 50  0001 C CNN
F 1 "+12V_ISO" H 1500 2940 50  0000 C CNN
F 2 "" H 1500 2800 50  0000 C CNN
F 3 "" H 1500 2800 50  0000 C CNN
	1    1500 2800
	1    0    0    -1  
$EndComp
Text HLabel 850  3750 0    60   Input ~ 0
VBUS
$Comp
L +12V_ISO #PWR015
U 1 1 59A9895F
P 2350 2800
F 0 "#PWR015" H 2350 2650 50  0001 C CNN
F 1 "+12V_ISO" H 2350 2940 50  0000 C CNN
F 2 "" H 2350 2800 50  0000 C CNN
F 3 "" H 2350 2800 50  0000 C CNN
	1    2350 2800
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR016
U 1 1 59A9BD78
P 850 1150
F 0 "#PWR016" H 850 1000 50  0001 C CNN
F 1 "+12V" H 850 1290 50  0000 C CNN
F 2 "" H 850 1150 50  0000 C CNN
F 3 "" H 850 1150 50  0000 C CNN
	1    850  1150
	1    0    0    -1  
$EndComp
$Comp
L -12V #PWR2
U 1 1 59A9BF35
P 850 1350
F 0 "#PWR2" H 850 1450 50  0001 C CNN
F 1 "-12V" H 850 1500 50  0000 C CNN
F 2 "" H 850 1350 50  0000 C CNN
F 3 "" H 850 1350 50  0000 C CNN
	1    850  1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 1050 1600 1050
Wire Wire Line
	850  1200 1700 1200
Wire Wire Line
	850  1300 1700 1300
Wire Wire Line
	2700 1250 2900 1250
Wire Wire Line
	2900 1250 2900 1450
Wire Wire Line
	2900 1750 2900 1850
Wire Wire Line
	2900 1800 3150 1800
Connection ~ 2900 1800
Wire Wire Line
	3150 1800 3150 1750
Wire Wire Line
	3150 1450 3150 1350
Wire Wire Line
	2900 1350 3700 1350
Connection ~ 2900 1350
Wire Wire Line
	4350 1750 4350 1800
Wire Wire Line
	4150 1550 4150 2050
Wire Wire Line
	4150 2050 4900 2050
Wire Wire Line
	4900 2050 4900 1450
Wire Wire Line
	4750 1450 5050 1450
Connection ~ 4900 1450
Wire Wire Line
	4350 1150 4350 1000
Wire Wire Line
	3600 1100 3600 1150
Wire Wire Line
	3600 1150 3700 1150
Wire Wire Line
	4000 1150 4050 1150
Wire Wire Line
	4050 1150 4050 1350
Wire Wire Line
	4000 1350 4150 1350
Connection ~ 4050 1350
Connection ~ 3150 1350
Wire Wire Line
	1000 7400 1000 7450
Wire Wire Line
	1000 7000 1000 7100
Wire Wire Line
	1000 6700 1000 6600
Wire Wire Line
	1000 7050 1350 7050
Connection ~ 1000 7050
Wire Wire Line
	1550 6600 1550 6650
Wire Wire Line
	1550 7250 1550 7300
Wire Wire Line
	1950 6950 2250 6950
Wire Wire Line
	2100 6950 2100 6400
Wire Wire Line
	2100 6400 1250 6400
Connection ~ 2100 6950
Wire Wire Line
	1350 6850 1250 6850
Wire Wire Line
	1250 6850 1250 6400
Wire Wire Line
	2350 2800 2350 3050
Wire Wire Line
	2350 3050 2450 3050
Wire Wire Line
	2300 3150 2450 3150
Wire Wire Line
	1900 3150 2000 3150
Wire Wire Line
	1500 3450 1500 3500
Wire Wire Line
	1500 2800 1500 2850
Wire Wire Line
	1200 3050 1300 3050
Wire Wire Line
	2450 3350 2450 3450
Wire Wire Line
	2450 3250 2200 3250
Wire Wire Line
	2200 3250 2200 3750
Wire Wire Line
	2200 3750 1200 3750
Wire Wire Line
	1250 3750 1250 3250
Wire Wire Line
	1250 3250 1300 3250
Connection ~ 1250 3750
Wire Wire Line
	900  3750 850  3750
Wire Wire Line
	1950 3650 1950 3750
Connection ~ 1950 3750
Wire Wire Line
	1950 3350 1950 3150
Connection ~ 1950 3150
Wire Wire Line
	3150 3250 3400 3250
Wire Wire Line
	3600 3450 3600 3500
Wire Wire Line
	3600 2800 3600 2850
Wire Wire Line
	3400 3050 3350 3050
Wire Wire Line
	3350 3050 3350 3350
Wire Wire Line
	3350 3350 3150 3350
Wire Wire Line
	4000 3150 4250 3150
Wire Wire Line
	4050 2350 4050 3150
Wire Wire Line
	4050 2500 3950 2500
Wire Wire Line
	3650 2500 3250 2500
Wire Wire Line
	3250 2350 3250 3250
Connection ~ 3250 3250
Wire Wire Line
	3300 2350 3250 2350
Connection ~ 3250 2500
Wire Wire Line
	3600 2350 4050 2350
Connection ~ 4050 2500
Connection ~ 4050 3150
Wire Wire Line
	850  1200 850  1150
Wire Wire Line
	850  1350 850  1300
Wire Wire Line
	2700 1050 2800 1050
$EndSCHEMATC
