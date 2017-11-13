EESchema Schematic File Version 2
LIBS:2kwMotorController-rescue
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
L LAH-100P U2
U 1 1 5975C20B
P 2150 1550
F 0 "U2" H 2150 1250 60  0000 C CNN
F 1 "LAH-100P" H 2150 1800 60  0000 C CNN
F 2 "2kwMotorController:LAH_100-P" H 2150 1550 60  0001 C CNN
F 3 "" H 2150 1550 60  0000 C CNN
	1    2150 1550
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5975C452
P 3100 2000
F 0 "R3" V 3180 2000 50  0000 C CNN
F 1 "59.0 0.1%" V 3100 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3030 2000 50  0001 C CNN
F 3 "" H 3100 2000 50  0000 C CNN
	1    3100 2000
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5975C496
P 3350 2000
F 0 "C2" H 3375 2100 50  0000 L CNN
F 1 "27n" H 3375 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3388 1850 50  0001 C CNN
F 3 "" H 3350 2000 50  0000 C CNN
F 4 "16V" H 3350 2000 60  0001 C CNN "Voltage"
	1    3350 2000
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5975C99C
P 3750 1750
F 0 "R5" V 3830 1750 50  0000 C CNN
F 1 "10k" V 3750 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3680 1750 50  0001 C CNN
F 3 "" H 3750 1750 50  0000 C CNN
	1    3750 1750
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 5975CA28
P 3750 1400
F 0 "R4" V 3830 1400 50  0000 C CNN
F 1 "10k" V 3750 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3680 1400 50  0001 C CNN
F 3 "" H 3750 1400 50  0000 C CNN
	1    3750 1400
	0    1    1    0   
$EndComp
Text Notes 1750 2100 0    60   ~ 0
IS_OUT <= 50mA\nBI-DIRECTIONAL
Text HLabel 1600 1450 0    60   BiDi ~ 12
I_SENSE+
Text HLabel 2700 1450 2    60   BiDi ~ 12
I_SENSE-
Text HLabel 5850 1850 2    60   Output ~ 12
I_SENSOR_OUT
Text Notes 3100 1000 2    118  ~ 24
motor current sensor
Text Notes 9450 3800 2    118  ~ 24
h-bridge temp sensor
Text Notes 4400 3300 2    118  ~ 24
controller battery voltage sensor
Text Notes 3950 5850 2    118  ~ 24
bus voltage sensor
Text Notes 2500 3550 2    60   ~ 0
6 - 9V
$Comp
L R R9
U 1 1 597CFA75
P 2350 4050
F 0 "R9" V 2430 4050 50  0000 C CNN
F 1 "270k" V 2350 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2280 4050 50  0001 C CNN
F 3 "" H 2350 4050 50  0000 C CNN
	1    2350 4050
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 597CFAC7
P 2350 4450
F 0 "R10" V 2430 4450 50  0000 C CNN
F 1 "100k" V 2350 4450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2280 4450 50  0001 C CNN
F 3 "" H 2350 4450 50  0000 C CNN
	1    2350 4450
	1    0    0    -1  
$EndComp
Text HLabel 2950 4250 2    60   Output ~ 12
VBAT_CTRL
Text HLabel 6100 6600 2    60   Output ~ 12
VBUS_MOTOR
$Comp
L R R2
U 1 1 597E87B7
P 3350 6600
F 0 "R2" V 3430 6600 50  0000 C CNN
F 1 "470" V 3350 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3280 6600 50  0001 C CNN
F 3 "" H 3350 6600 50  0000 C CNN
	1    3350 6600
	0    1    1    0   
$EndComp
$Comp
L LOC110 U3
U 1 1 597E8D03
P 4000 6650
F 0 "U3" H 4000 6400 60  0000 C CNN
F 1 "LOC110" H 4000 6900 60  0000 C CNN
F 2 "SMD_Packages:DIP-8_SMD" H 4150 6450 60  0001 C CNN
F 3 "" H 4150 6450 60  0000 C CNN
	1    4000 6650
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 597E9880
P 2250 7200
F 0 "R1" V 2330 7200 50  0000 C CNN
F 1 "120k" V 2250 7200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2180 7200 50  0001 C CNN
F 3 "" H 2250 7200 50  0000 C CNN
	1    2250 7200
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 597E9DB4
P 3150 6950
F 0 "C1" H 3175 7050 50  0000 L CNN
F 1 "10p" H 3175 6850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3188 6800 50  0001 C CNN
F 3 "" H 3150 6950 50  0000 C CNN
F 4 "16V" H 3150 6950 60  0001 C CNN "Voltage"
	1    3150 6950
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U4
U 1 1 597EAA48
P 4900 6600
F 0 "U4" H 4950 6800 50  0000 C CNN
F 1 "MCP6001" H 5100 6400 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SC-70-5" H 4850 6700 50  0001 C CNN
F 3 "" H 4950 6800 50  0000 C CNN
	1    4900 6600
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 597EB265
P 5200 6000
F 0 "R6" V 5280 6000 50  0000 C CNN
F 1 "39k" V 5200 6000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5130 6000 50  0001 C CNN
F 3 "" H 5200 6000 50  0000 C CNN
	1    5200 6000
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 597EB397
P 5300 6250
F 0 "C3" V 5250 6350 50  0000 L CNN
F 1 "10p" V 5250 6050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5338 6100 50  0001 C CNN
F 3 "" H 5300 6250 50  0000 C CNN
F 4 "16V" V 5300 6250 60  0001 C CNN "Voltage"
	1    5300 6250
	0    -1   -1   0   
$EndComp
$Comp
L +12V_ISO #PWR01
U 1 1 59A951A8
P 2700 6250
F 0 "#PWR01" H 2700 6100 50  0001 C CNN
F 1 "+12V_ISO" H 2700 6390 50  0000 C CNN
F 2 "" H 2700 6250 50  0000 C CNN
F 3 "" H 2700 6250 50  0000 C CNN
	1    2700 6250
	1    0    0    -1  
$EndComp
Text HLabel 950  6300 0    60   Input ~ 12
VBUS
$Comp
L +12V_ISO #PWR02
U 1 1 59A9895F
P 3550 6250
F 0 "#PWR02" H 3550 6100 50  0001 C CNN
F 1 "+12V_ISO" H 3550 6390 50  0000 C CNN
F 2 "" H 3550 6250 50  0000 C CNN
F 3 "" H 3550 6250 50  0000 C CNN
	1    3550 6250
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR03
U 1 1 59A9BD78
P 950 1550
F 0 "#PWR03" H 950 1400 50  0001 C CNN
F 1 "+12V" H 950 1690 50  0000 C CNN
F 2 "" H 950 1550 50  0000 C CNN
F 3 "" H 950 1550 50  0000 C CNN
	1    950  1550
	1    0    0    -1  
$EndComp
$Comp
L -12V #PWR2
U 1 1 59A9BF35
P 950 2050
F 0 "#PWR2" H 950 2150 50  0001 C CNN
F 1 "-12V" H 950 2200 50  0000 C CNN
F 2 "" H 950 2050 50  0000 C CNN
F 3 "" H 950 2050 50  0000 C CNN
	1    950  2050
	-1   0    0    1   
$EndComp
Text Notes 5600 6250 0    60   ~ 0
optocoupler CTR:\nk3 = [0.7, 1.2] (ideally 1.0)
$Comp
L POT-RESCUE-2kwMotorController RV2
U 1 1 599D4CE6
P 5550 6000
F 0 "RV2" H 5550 5920 50  0000 C CNN
F 1 "36k" H 5550 6000 50  0000 C CNN
F 2 "2kwMotorController:Trimpot-23B" H 5550 6000 50  0001 C CNN
F 3 "" H 5550 6000 50  0000 C CNN
	1    5550 6000
	1    0    0    1   
$EndComp
$Comp
L POT-RESCUE-2kwMotorController RV1
U 1 1 59A7AA70
P 4100 1550
F 0 "RV1" H 4100 1470 50  0000 C CNN
F 1 "100" H 4100 1550 50  0000 C CNN
F 2 "2kwMotorController:Trimpot-23B" H 4100 1550 50  0001 C CNN
F 3 "" H 4100 1550 50  0000 C CNN
	1    4100 1550
	1    0    0    -1  
$EndComp
Text Notes 5300 950  0    60   ~ 0
rflt, cflt to drive SAR ADC input:\nRadc <= 2.5k ohm\nCadc <= 10pF  
Text Notes 5100 1500 0    60   ~ 0
I_motor = (Vout - 1.50) * 67.8\nbandwidth = 100kHz
Text Notes 2700 2100 0    60   ~ 0
0.05%\n180mW
$Comp
L +3VA #PWR04
U 1 1 59C015DC
P 3550 1250
F 0 "#PWR04" H 3550 1100 50  0001 C CNN
F 1 "+3VA" H 3550 1390 50  0000 C CNN
F 2 "" H 3550 1250 50  0000 C CNN
F 3 "" H 3550 1250 50  0000 C CNN
	1    3550 1250
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR05
U 1 1 59C0683C
P 2700 6950
F 0 "#PWR05" H 2700 6750 50  0001 C CNN
F 1 "GNDPWR" H 2700 6820 50  0000 C CNN
F 2 "" H 2700 6900 50  0000 C CNN
F 3 "" H 2700 6900 50  0000 C CNN
	1    2700 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 59C0EF65
P 2700 4700
F 0 "#PWR06" H 2700 4450 50  0001 C CNN
F 1 "GND" H 2700 4550 50  0000 C CNN
F 2 "" H 2700 4700 50  0000 C CNN
F 3 "" H 2700 4700 50  0000 C CNN
	1    2700 4700
	1    0    0    -1  
$EndComp
Text HLabel 8900 2100 2    60   Output ~ 0
MOTOR_TEMP
Text HLabel 9050 4550 2    60   Output ~ 0
BRIDGE_TEMP
Text Notes 2600 2650 0    60   ~ 0
Use good tempco resistor\nand provide thermal relief
$Comp
L R R8
U 1 1 59B94C38
P 5450 1850
F 0 "R8" V 5530 1850 50  0000 C CNN
F 1 "33" V 5450 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5380 1850 50  0001 C CNN
F 3 "" H 5450 1850 50  0000 C CNN
	1    5450 1850
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 59B94C8F
P 5650 2050
F 0 "C5" H 5675 2150 50  0000 L CNN
F 1 "47n" H 5675 1950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5688 1900 50  0001 C CNN
F 3 "" H 5650 2050 50  0000 C CNN
F 4 "16V" H 5650 2050 60  0001 C CNN "Voltage"
	1    5650 2050
	1    0    0    -1  
$EndComp
$Comp
L ADA4805-1 U5
U 1 1 59B96B07
P 4700 1850
F 0 "U5" H 4750 2050 50  0000 C CNN
F 1 "ADA4805-1" H 4900 1650 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 4650 1950 50  0001 C CNN
F 3 "" H 4750 2050 50  0000 C CNN
	1    4700 1850
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 59B9903C
P 5750 6600
F 0 "R7" V 5830 6600 50  0000 C CNN
F 1 "47" V 5750 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5680 6600 50  0001 C CNN
F 3 "" H 5750 6600 50  0000 C CNN
	1    5750 6600
	0    1    1    0   
$EndComp
$Comp
L C C4
U 1 1 59B990C0
P 5950 6800
F 0 "C4" H 5975 6900 50  0000 L CNN
F 1 "100n" H 5975 6700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5988 6650 50  0001 C CNN
F 3 "" H 5950 6800 50  0000 C CNN
F 4 "16V" H 5950 6800 60  0001 C CNN "Voltage"
	1    5950 6800
	1    0    0    -1  
$EndComp
$Comp
L R R38
U 1 1 59BEC59E
P 8300 1900
F 0 "R38" V 8380 1900 50  0000 C CNN
F 1 "10k" V 8300 1900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 1900 50  0001 C CNN
F 3 "" H 8300 1900 50  0000 C CNN
	1    8300 1900
	1    0    0    -1  
$EndComp
$Comp
L C C76
U 1 1 59BED994
P 8600 2350
F 0 "C76" H 8625 2450 50  0000 L CNN
F 1 "100n" H 8625 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8638 2200 50  0001 C CNN
F 3 "" H 8600 2350 50  0000 C CNN
F 4 "16V" H 8600 2350 60  0001 C CNN "Voltage"
	1    8600 2350
	1    0    0    -1  
$EndComp
$Comp
L +3V3A #PWR07
U 1 1 59BF15F2
P 4800 5600
F 0 "#PWR07" H 4800 5450 50  0001 C CNN
F 1 "+3V3A" H 4800 5740 50  0000 C CNN
F 2 "" H 4800 5600 50  0000 C CNN
F 3 "" H 4800 5600 50  0000 C CNN
	1    4800 5600
	1    0    0    -1  
$EndComp
$Comp
L +3V3A #PWR08
U 1 1 59BF16DA
P 4600 1200
F 0 "#PWR08" H 4600 1050 50  0001 C CNN
F 1 "+3V3A" H 4600 1340 50  0000 C CNN
F 2 "" H 4600 1200 50  0000 C CNN
F 3 "" H 4600 1200 50  0000 C CNN
	1    4600 1200
	1    0    0    -1  
$EndComp
$Comp
L +3V3A #PWR09
U 1 1 59BF22A3
P 8300 1650
F 0 "#PWR09" H 8300 1500 50  0001 C CNN
F 1 "+3V3A" H 8300 1790 50  0000 C CNN
F 2 "" H 8300 1650 50  0000 C CNN
F 3 "" H 8300 1650 50  0000 C CNN
	1    8300 1650
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR010
U 1 1 59BF36AA
P 2350 3800
F 0 "#PWR010" H 2350 3650 50  0001 C CNN
F 1 "+BATT" H 2350 3940 50  0000 C CNN
F 2 "" H 2350 3800 50  0000 C CNN
F 3 "" H 2350 3800 50  0000 C CNN
	1    2350 3800
	1    0    0    -1  
$EndComp
$Comp
L C_Small C75
U 1 1 59BFB78B
P 4850 1400
F 0 "C75" H 4860 1470 50  0000 L CNN
F 1 "100n" H 4860 1320 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4850 1400 50  0001 C CNN
F 3 "" H 4850 1400 50  0000 C CNN
F 4 "16V" H 4850 1400 60  0001 C CNN "Voltage"
	1    4850 1400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C74
U 1 1 59C06005
P 4600 5750
F 0 "C74" H 4610 5820 50  0000 L CNN
F 1 "100n" H 4610 5670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4600 5750 50  0001 C CNN
F 3 "" H 4600 5750 50  0000 C CNN
F 4 "16V" H 4600 5750 60  0001 C CNN "Voltage"
	1    4600 5750
	1    0    0    -1  
$EndComp
$Comp
L R R59
U 1 1 59C18B68
P 8300 4350
F 0 "R59" V 8380 4350 50  0000 C CNN
F 1 "10k" V 8300 4350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 4350 50  0001 C CNN
F 3 "" H 8300 4350 50  0000 C CNN
	1    8300 4350
	1    0    0    -1  
$EndComp
$Comp
L C C77
U 1 1 59C18B71
P 8600 4800
F 0 "C77" H 8625 4900 50  0000 L CNN
F 1 "100n" H 8625 4700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8638 4650 50  0001 C CNN
F 3 "" H 8600 4800 50  0000 C CNN
F 4 "16V" H 8600 4800 60  0001 C CNN "Voltage"
	1    8600 4800
	1    0    0    -1  
$EndComp
$Comp
L +3V3A #PWR011
U 1 1 59C18B81
P 8300 4100
F 0 "#PWR011" H 8300 3950 50  0001 C CNN
F 1 "+3V3A" H 8300 4240 50  0000 C CNN
F 2 "" H 8300 4100 50  0000 C CNN
F 3 "" H 8300 4100 50  0000 C CNN
	1    8300 4100
	1    0    0    -1  
$EndComp
Text Notes 9200 1350 2    118  ~ 24
motor temp sensor
$Comp
L C C6
U 1 1 59C5BCF9
P 2700 4450
F 0 "C6" H 2725 4550 50  0000 L CNN
F 1 "100n" H 2725 4350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2738 4300 50  0001 C CNN
F 3 "" H 2700 4450 50  0000 C CNN
F 4 "16V" H 2700 4450 60  0001 C CNN "Voltage"
	1    2700 4450
	1    0    0    -1  
$EndComp
NoConn ~ 5700 6000
$Comp
L C C73
U 1 1 59CB7B98
P 950 1800
F 0 "C73" H 975 1900 50  0000 L CNN
F 1 "100n" H 975 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 988 1650 50  0001 C CNN
F 3 "" H 950 1800 50  0000 C CNN
F 4 "25V" H 950 1800 60  0001 C CNN "Voltage"
	1    950  1800
	1    0    0    -1  
$EndComp
$Comp
L C C78
U 1 1 59CB93D8
P 1250 1800
F 0 "C78" H 1275 1900 50  0000 L CNN
F 1 "1u" H 1275 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1288 1650 50  0001 C CNN
F 3 "" H 1250 1800 50  0000 C CNN
F 4 "25V" H 1250 1800 60  0001 C CNN "Voltage"
	1    1250 1800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P13
U 1 1 59CD3AE3
P 8100 2400
F 0 "P13" H 8100 2550 50  0000 C CNN
F 1 "CONN_01X02" H 8275 2225 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 8100 2400 50  0001 C CNN
F 3 "" H 8100 2400 50  0000 C CNN
F 4 "MOTOR_TEMP" H 8300 2125 60  0000 C CNN "Use"
	1    8100 2400
	-1   0    0    1   
$EndComp
Text Notes 7075 2575 0    60   ~ 0
NTC THERMISTOR\nPROBE OFF-BOARD:\nNXRT15WB473FA1B
Text Notes 7075 5025 0    60   ~ 0
NTC THERMISTOR\nPROBE OFF-BOARD:\nNXRT15WB473FA1B
$Comp
L CONN_01X02 P14
U 1 1 59CD58D2
P 8100 4850
F 0 "P14" H 8100 5000 50  0000 C CNN
F 1 "CONN_01X02" H 8275 4675 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 8100 4850 50  0001 C CNN
F 3 "" H 8100 4850 50  0000 C CNN
F 4 "H-BRIDGE TEMP" H 8375 4600 60  0000 C CNN "Use"
	1    8100 4850
	-1   0    0    1   
$EndComp
NoConn ~ 3950 1550
$Comp
L GND #PWR012
U 1 1 59C15774
P 4800 7400
F 0 "#PWR012" H 4800 7150 50  0001 C CNN
F 1 "GND" H 4800 7250 50  0000 C CNN
F 2 "" H 4800 7400 50  0000 C CNN
F 3 "" H 4800 7400 50  0000 C CNN
	1    4800 7400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 59C157E9
P 8600 5200
F 0 "#PWR013" H 8600 4950 50  0001 C CNN
F 1 "GND" H 8600 5050 50  0000 C CNN
F 2 "" H 8600 5200 50  0000 C CNN
F 3 "" H 8600 5200 50  0000 C CNN
	1    8600 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 59C15D06
P 8600 2750
F 0 "#PWR014" H 8600 2500 50  0001 C CNN
F 1 "GND" H 8600 2600 50  0000 C CNN
F 2 "" H 8600 2750 50  0000 C CNN
F 3 "" H 8600 2750 50  0000 C CNN
	1    8600 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 59C16167
P 5650 2250
F 0 "#PWR015" H 5650 2000 50  0001 C CNN
F 1 "GND" H 5650 2100 50  0000 C CNN
F 2 "" H 5650 2250 50  0000 C CNN
F 3 "" H 5650 2250 50  0000 C CNN
	1    5650 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 59C169DF
P 4600 2250
F 0 "#PWR016" H 4600 2000 50  0001 C CNN
F 1 "GND" H 4600 2100 50  0000 C CNN
F 2 "" H 4600 2250 50  0000 C CNN
F 3 "" H 4600 2250 50  0000 C CNN
	1    4600 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 59C16EAB
P 3100 2250
F 0 "#PWR017" H 3100 2000 50  0001 C CNN
F 1 "GND" H 3100 2100 50  0000 C CNN
F 2 "" H 3100 2250 50  0000 C CNN
F 3 "" H 3100 2250 50  0000 C CNN
	1    3100 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 59C17E29
P 4600 5900
F 0 "#PWR018" H 4600 5650 50  0001 C CNN
F 1 "GND" H 4600 5750 50  0000 C CNN
F 2 "" H 4600 5900 50  0000 C CNN
F 3 "" H 4600 5900 50  0000 C CNN
	1    4600 5900
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR019
U 1 1 59C883AE
P 2250 6550
F 0 "#PWR019" H 2250 6350 50  0001 C CNN
F 1 "GNDPWR" H 2250 6420 50  0000 C CNN
F 2 "" H 2250 6500 50  0000 C CNN
F 3 "" H 2250 6500 50  0000 C CNN
	1    2250 6550
	1    0    0    -1  
$EndComp
$Comp
L LM358 U1
U 2 1 59C89C78
P 2800 6600
F 0 "U1" H 2750 6800 50  0000 L CNN
F 1 "LM358A" H 2750 6350 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2800 6600 50  0001 C CNN
F 3 "" H 2800 6600 50  0000 C CNN
	2    2800 6600
	1    0    0    -1  
$EndComp
$Comp
L R R44
U 1 1 59C94773
P 1000 6550
F 0 "R44" V 1080 6550 50  0000 C CNN
F 1 "18k" V 1000 6550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 930 6550 50  0001 C CNN
F 3 "" H 1000 6550 50  0000 C CNN
	1    1000 6550
	1    0    0    -1  
$EndComp
$Comp
L R R45
U 1 1 59C94819
P 1000 6950
F 0 "R45" V 1080 6950 50  0000 C CNN
F 1 "2k" V 1000 6950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 930 6950 50  0001 C CNN
F 3 "" H 1000 6950 50  0000 C CNN
	1    1000 6950
	1    0    0    -1  
$EndComp
$Comp
L +12V_ISO #PWR020
U 1 1 59C94D9E
P 1450 6300
F 0 "#PWR020" H 1450 6150 50  0001 C CNN
F 1 "+12V_ISO" H 1450 6440 50  0000 C CNN
F 2 "" H 1450 6300 50  0000 C CNN
F 3 "" H 1450 6300 50  0000 C CNN
	1    1450 6300
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR021
U 1 1 59C067BD
P 3650 7400
F 0 "#PWR021" H 3650 7200 50  0001 C CNN
F 1 "GNDPWR" H 3650 7270 50  0000 C CNN
F 2 "" H 3650 7350 50  0000 C CNN
F 3 "" H 3650 7350 50  0000 C CNN
	1    3650 7400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR022
U 1 1 59C93E25
P 1450 7400
F 0 "#PWR022" H 1450 7200 50  0001 C CNN
F 1 "GNDPWR" H 1450 7270 50  0000 C CNN
F 2 "" H 1450 7350 50  0000 C CNN
F 3 "" H 1450 7350 50  0000 C CNN
	1    1450 7400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR023
U 1 1 59C94A17
P 1000 7400
F 0 "#PWR023" H 1000 7200 50  0001 C CNN
F 1 "GNDPWR" H 1000 7270 50  0000 C CNN
F 2 "" H 1000 7350 50  0000 C CNN
F 3 "" H 1000 7350 50  0000 C CNN
	1    1000 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1450 1600 1450
Wire Wire Line
	950  1600 1650 1600
Wire Wire Line
	2650 1650 3100 1650
Wire Wire Line
	3100 1650 3100 1850
Wire Wire Line
	3100 2150 3100 2250
Wire Wire Line
	3100 2200 3350 2200
Connection ~ 3100 2200
Wire Wire Line
	3350 2200 3350 2150
Wire Wire Line
	3350 1700 3350 1850
Wire Wire Line
	3100 1750 3600 1750
Connection ~ 3100 1750
Wire Wire Line
	4600 2150 4600 2250
Wire Wire Line
	4400 1950 4400 2450
Wire Wire Line
	4400 2450 5150 2450
Wire Wire Line
	5150 2450 5150 1850
Wire Wire Line
	5000 1850 5300 1850
Connection ~ 5150 1850
Wire Wire Line
	4600 1200 4600 1550
Wire Wire Line
	3550 1250 3550 1400
Wire Wire Line
	3550 1400 3600 1400
Wire Wire Line
	4350 1200 4350 1750
Wire Wire Line
	3900 1750 4400 1750
Connection ~ 4350 1750
Connection ~ 3350 1750
Wire Wire Line
	2350 4600 2350 4650
Wire Wire Line
	2350 4200 2350 4300
Wire Wire Line
	2350 3900 2350 3800
Connection ~ 2350 4250
Wire Wire Line
	3550 6250 3550 6500
Wire Wire Line
	3550 6500 3650 6500
Wire Wire Line
	3500 6600 3650 6600
Wire Wire Line
	3100 6600 3200 6600
Wire Wire Line
	2700 6900 2700 6950
Wire Wire Line
	2700 6250 2700 6300
Wire Wire Line
	2250 6500 2500 6500
Wire Wire Line
	3650 6800 3650 7400
Wire Wire Line
	3650 6700 3400 6700
Wire Wire Line
	3400 6700 3400 7200
Wire Wire Line
	3400 7200 2400 7200
Wire Wire Line
	2450 7200 2450 6700
Wire Wire Line
	2450 6700 2500 6700
Connection ~ 2450 7200
Wire Wire Line
	1200 7200 2100 7200
Wire Wire Line
	3150 7100 3150 7200
Connection ~ 3150 7200
Wire Wire Line
	3150 6800 3150 6600
Connection ~ 3150 6600
Wire Wire Line
	4350 6700 4600 6700
Wire Wire Line
	4800 6900 4800 7400
Wire Wire Line
	4600 6500 4550 6500
Wire Wire Line
	4550 6500 4550 7200
Wire Wire Line
	4550 6800 4350 6800
Wire Wire Line
	5200 6600 5600 6600
Wire Wire Line
	4450 6250 5150 6250
Wire Wire Line
	5000 6250 5000 6000
Wire Wire Line
	4450 6250 4450 6700
Connection ~ 4450 6700
Wire Wire Line
	950  1550 950  1650
Wire Wire Line
	2650 1450 2700 1450
Connection ~ 4550 6800
Connection ~ 5550 6600
Wire Wire Line
	3900 1400 4100 1400
Wire Wire Line
	4250 1550 4350 1550
Wire Wire Line
	5600 1850 5850 1850
Wire Wire Line
	5650 1800 5650 1900
Wire Wire Line
	5650 2200 5650 2250
Connection ~ 5650 1850
Wire Wire Line
	4600 1250 4850 1250
Wire Wire Line
	4700 1250 4700 1600
Connection ~ 4600 1250
Wire Wire Line
	5950 7200 5950 6950
Wire Wire Line
	5900 6600 6100 6600
Wire Wire Line
	5950 6550 5950 6650
Connection ~ 5950 6600
Wire Wire Line
	8300 2450 8300 2700
Wire Wire Line
	8300 2050 8300 2350
Wire Wire Line
	8300 1750 8300 1650
Wire Wire Line
	8300 2100 8900 2100
Wire Wire Line
	8600 2050 8600 2200
Connection ~ 8300 2100
Wire Wire Line
	8600 2500 8600 2750
Connection ~ 8600 2100
Connection ~ 8600 2700
Wire Wire Line
	4850 1550 4850 1500
Connection ~ 4700 1250
Wire Wire Line
	4850 1250 4850 1300
Wire Wire Line
	4550 7200 5950 7200
Connection ~ 4800 7200
Wire Wire Line
	4600 5600 4800 5600
Wire Wire Line
	4800 5600 4800 6300
Wire Wire Line
	5550 6150 5550 6600
Wire Wire Line
	5350 6000 5400 6000
Connection ~ 5000 6250
Wire Wire Line
	5450 6250 5550 6250
Connection ~ 5550 6250
Wire Wire Line
	4600 5650 4600 5600
Wire Wire Line
	4600 5900 4600 5850
Wire Wire Line
	5000 6000 5050 6000
Wire Wire Line
	8300 4900 8300 5150
Wire Wire Line
	8300 4500 8300 4800
Wire Wire Line
	8300 4200 8300 4100
Wire Wire Line
	8600 4500 8600 4650
Connection ~ 8300 4550
Wire Wire Line
	8600 4950 8600 5200
Connection ~ 8600 4550
Connection ~ 8600 5150
Wire Wire Line
	8300 2700 8600 2700
Wire Wire Line
	8300 4550 9050 4550
Wire Wire Line
	8300 5150 8600 5150
Wire Wire Line
	2350 4250 2950 4250
Wire Wire Line
	2700 4250 2700 4300
Wire Wire Line
	2700 4600 2700 4700
Wire Wire Line
	2350 4650 2700 4650
Connection ~ 2700 4650
Connection ~ 2700 4250
Connection ~ 950  1600
Wire Wire Line
	950  1950 950  2050
Wire Wire Line
	950  2000 1550 2000
Wire Wire Line
	1550 2000 1550 1700
Wire Wire Line
	1550 1700 1650 1700
Connection ~ 950  2000
Wire Wire Line
	1250 1650 1250 1600
Connection ~ 1250 1600
Wire Wire Line
	1250 1950 1250 2000
Connection ~ 1250 2000
Wire Wire Line
	1450 7400 1450 7150
Wire Wire Line
	1250 6950 1200 6950
Wire Wire Line
	1200 6950 1200 7200
Wire Wire Line
	1900 6800 1900 7200
Wire Wire Line
	1850 6850 1900 6850
Wire Wire Line
	1900 6850 1950 6900
Wire Wire Line
	1000 6700 1000 6800
Wire Wire Line
	1000 6750 1250 6750
Connection ~ 1000 6750
Wire Wire Line
	1000 7400 1000 7100
Wire Wire Line
	1450 6300 1450 6550
Wire Wire Line
	1000 6300 1000 6400
Connection ~ 1450 6350
Wire Wire Line
	950  6300 1000 6300
Connection ~ 1900 7200
Wire Wire Line
	2250 6550 2250 6500
Connection ~ 1900 6850
Text Label 1950 6900 0    60   ~ 0
VBUS_DIV
Text Label 2675 1650 0    60   ~ 0
ISENSE_OUT
Connection ~ 4350 1550
Text Label 4300 1200 2    60   ~ 0
IS_SUM
Wire Wire Line
	4350 1250 4300 1200
Connection ~ 4350 1250
$Comp
L LM358 U1
U 1 1 59C89BF1
P 1550 6850
F 0 "U1" H 1500 7050 50  0000 L CNN
F 1 "LM358A" H 1500 6600 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1550 6850 50  0001 C CNN
F 3 "" H 1550 6850 50  0000 C CNN
	1    1550 6850
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP1
U 1 1 59C9BBF4
P 1900 6800
F 0 "TP1" H 1900 7000 60  0000 C CNN
F 1 "TEST_POINT" H 1900 7200 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 1900 6800 60  0001 C CNN
F 3 "" H 1900 6800 60  0000 C CNN
	1    1900 6800
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP6
U 1 1 59C9C25D
P 5950 6550
F 0 "TP6" H 5950 6750 60  0000 C CNN
F 1 "TEST_POINT" H 5950 6950 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 5950 6550 60  0001 C CNN
F 3 "" H 5950 6550 60  0000 C CNN
	1    5950 6550
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP2
U 1 1 59C9CAB5
P 2700 4250
F 0 "TP2" H 2700 4450 60  0000 C CNN
F 1 "TEST_POINT" H 2700 4650 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 2700 4250 60  0001 C CNN
F 3 "" H 2700 4250 60  0000 C CNN
	1    2700 4250
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP3
U 1 1 59C9D1E0
P 3350 1700
F 0 "TP3" H 3350 1900 60  0000 C CNN
F 1 "TEST_POINT" H 3350 2100 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 3350 1700 60  0001 C CNN
F 3 "" H 3350 1700 60  0000 C CNN
	1    3350 1700
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP4
U 1 1 59C9E501
P 4350 1200
F 0 "TP4" H 4350 1400 60  0000 C CNN
F 1 "TEST_POINT" H 4350 1600 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 4350 1200 60  0001 C CNN
F 3 "" H 4350 1200 60  0000 C CNN
	1    4350 1200
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP5
U 1 1 59C9F20E
P 5650 1800
F 0 "TP5" H 5650 2000 60  0000 C CNN
F 1 "TEST_POINT" H 5650 2200 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 5650 1800 60  0001 C CNN
F 3 "" H 5650 1800 60  0000 C CNN
	1    5650 1800
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP7
U 1 1 59C9FA31
P 8600 2050
F 0 "TP7" H 8600 2250 60  0000 C CNN
F 1 "TEST_POINT" H 8600 2450 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 8600 2050 60  0001 C CNN
F 3 "" H 8600 2050 60  0000 C CNN
	1    8600 2050
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP8
U 1 1 59CA0847
P 8600 4500
F 0 "TP8" H 8600 4700 60  0000 C CNN
F 1 "TEST_POINT" H 8600 4900 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 8600 4500 60  0001 C CNN
F 3 "" H 8600 4500 60  0000 C CNN
	1    8600 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 59CC5A0B
P 4850 1550
F 0 "#PWR024" H 4850 1300 50  0001 C CNN
F 1 "GND" H 4850 1400 50  0000 C CNN
F 2 "" H 4850 1550 50  0000 C CNN
F 3 "" H 4850 1550 50  0000 C CNN
	1    4850 1550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
