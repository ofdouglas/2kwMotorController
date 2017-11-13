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
Sheet 3 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 "Author: Oliver Douglas"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X02 P1
U 1 1 597A960B
P 950 1200
F 0 "P1" H 950 1350 50  0000 C CNN
F 1 "CONN_01X02" H 950 1025 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 950 1200 50  0001 C CNN
F 3 "" H 950 1200 50  0000 C CNN
F 4 "CONTROLLER BATTERY" H 925 950 60  0000 C CNN "Use"
	1    950  1200
	-1   0    0    1   
$EndComp
Text Notes 800  850  0    60   ~ 12
Battery Input\n9V max / 5V min\n750 mA max load\n2S LiPo Recommended
$Comp
L GND #PWR025
U 1 1 597A9AEF
P 2600 7300
F 0 "#PWR025" H 2600 7050 50  0001 C CNN
F 1 "GND" H 2600 7150 50  0000 C CNN
F 2 "" H 2600 7300 50  0000 C CNN
F 3 "" H 2600 7300 50  0000 C CNN
	1    2600 7300
	1    0    0    -1  
$EndComp
$Comp
L PDM1-S U11
U 1 1 59858184
P 7800 4400
F 0 "U11" H 7850 4200 59  0000 C CNN
F 1 "PDM1-S12-S3" H 7850 4600 59  0000 C CNN
F 2 "2kwMotorController:PDM2-S" H 7850 4400 59  0001 C CNN
F 3 "" H 7850 4400 59  0000 C CNN
	1    7800 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR026
U 1 1 598582EF
P 8200 5200
F 0 "#PWR026" H 8200 5000 50  0001 C CNN
F 1 "GNDPWR" H 8200 5070 50  0000 C CNN
F 2 "" H 8200 5150 50  0000 C CNN
F 3 "" H 8200 5150 50  0000 C CNN
	1    8200 5200
	1    0    0    -1  
$EndComp
$Comp
L R R23
U 1 1 598583E6
P 8750 4950
F 0 "R23" V 8830 4950 50  0000 C CNN
F 1 "1.8k" V 8750 4950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8680 4950 50  0001 C CNN
F 3 "" H 8750 4950 50  0000 C CNN
	1    8750 4950
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-2kwMotorController D7
U 1 1 5985847D
P 8750 4550
F 0 "D7" H 8750 4650 50  0000 C CNN
F 1 "LED" H 8750 4450 50  0000 C CNN
F 2 "LEDs:LED_0603" H 8750 4550 50  0001 C CNN
F 3 "" H 8750 4550 50  0000 C CNN
	1    8750 4550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR027
U 1 1 59858B6A
P 7400 5150
F 0 "#PWR027" H 7400 4900 50  0001 C CNN
F 1 "GND" H 7400 5000 50  0000 C CNN
F 2 "" H 7400 5150 50  0000 C CNN
F 3 "" H 7400 5150 50  0000 C CNN
	1    7400 5150
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 59870B9B
P 2100 6750
F 0 "R12" V 2180 6750 50  0000 C CNN
F 1 "100" V 2100 6750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2030 6750 50  0001 C CNN
F 3 "" H 2100 6750 50  0000 C CNN
	1    2100 6750
	-1   0    0    1   
$EndComp
Text Notes 2750 7700 2    59   ~ 12
REVERSE POLARITY PROTECTION\n14mW max - no heatsink needed
Text Notes 8100 5650 0    60   ~ 12
H-BRIDGE\nGROUND \n
Text Notes 7100 5650 0    60   ~ 12
CONTROLLER\nGROUND 
Text Notes 7450 4050 0    79   ~ 16
ISOLATED DC/DC\nCONVERTER
$Comp
L LT3471 U8
U 1 1 5990AD2A
P 3500 5400
F 0 "U8" H 3250 4750 60  0000 C CNN
F 1 "LT3471" H 3500 6000 60  0000 C CNN
F 2 "Housings_DFN_QFN:DFN-10-1EP_3x3mm_Pitch0.5mm" H 3650 5500 60  0001 C CNN
F 3 "" H 3650 5500 60  0000 C CNN
	1    3500 5400
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L1
U 1 1 5990CBEA
P 3750 4350
F 0 "L1" H 3750 4450 50  0000 C CNN
F 1 "10u" H 3750 4300 50  0000 C CNN
F 2 "2kwMotorController:INDUCTOR_NR40xx" H 3750 4350 50  0001 C CNN
F 3 "NR4018T100M" H 3750 4200 50  0000 C CNN
	1    3750 4350
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D2
U 1 1 5990D0FB
P 4400 4350
F 0 "D2" H 4400 4450 50  0000 C CNN
F 1 "MBRM-120" H 4400 4250 50  0000 C CNN
F 2 "2kwMotorController:DIODE_POWERMITE" H 4400 4350 50  0001 C CNN
F 3 "" H 4400 4350 50  0000 C CNN
	1    4400 4350
	-1   0    0    1   
$EndComp
$Comp
L R R16
U 1 1 5990EC1D
P 4850 4700
F 0 "R16" V 4930 4700 50  0000 C CNN
F 1 "110k" V 4850 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4780 4700 50  0001 C CNN
F 3 "" H 4850 4700 50  0000 C CNN
	1    4850 4700
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 5990EC8C
P 5100 5050
F 0 "R17" V 5180 5050 50  0000 C CNN
F 1 "10k" V 5100 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5030 5050 50  0001 C CNN
F 3 "" H 5100 5050 50  0000 C CNN
	1    5100 5050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR028
U 1 1 5990F37C
P 5450 5100
F 0 "#PWR028" H 5450 4850 50  0001 C CNN
F 1 "GND" H 5450 4950 50  0000 C CNN
F 2 "" H 5450 5100 50  0000 C CNN
F 3 "" H 5450 5100 50  0000 C CNN
	1    5450 5100
	1    0    0    -1  
$EndComp
$Comp
L C C18
U 1 1 5990F905
P 5450 4550
F 0 "C18" H 5475 4650 50  0000 L CNN
F 1 "10u" H 5475 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5488 4400 50  0001 C CNN
F 3 "" H 5450 4550 50  0000 C CNN
F 4 "25V" H 5450 4550 60  0001 C CNN "Voltage"
	1    5450 4550
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L2
U 1 1 59910EF5
P 3850 6550
F 0 "L2" H 3750 6650 50  0000 C CNN
F 1 "15u" H 3950 6650 50  0000 C CNN
F 2 "2kwMotorController:INDUCTOR_NR40xx" H 3850 6550 50  0001 C CNN
F 3 "NR4012T150M" H 3850 6500 50  0000 C CNN
	1    3850 6550
	1    0    0    1   
$EndComp
$Comp
L C C13
U 1 1 59911761
P 4350 6550
F 0 "C13" H 4375 6650 50  0000 L CNN
F 1 "1u" H 4375 6450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4388 6400 50  0001 C CNN
F 3 "" H 4350 6550 50  0000 C CNN
	1    4350 6550
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D3
U 1 1 59911DDA
P 4550 6850
F 0 "D3" H 4550 6950 50  0000 C CNN
F 1 "MBRM-120" H 4550 6750 50  0000 C CNN
F 2 "2kwMotorController:DIODE_POWERMITE" H 4550 6850 50  0001 C CNN
F 3 "" H 4550 6850 50  0000 C CNN
	1    4550 6850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR029
U 1 1 5991235D
P 4550 7050
F 0 "#PWR029" H 4550 6800 50  0001 C CNN
F 1 "GND" H 4550 6900 50  0000 C CNN
F 2 "" H 4550 7050 50  0000 C CNN
F 3 "" H 4550 7050 50  0000 C CNN
	1    4550 7050
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 59913546
P 5200 6850
F 0 "C17" H 5225 6950 50  0000 L CNN
F 1 "10u" H 5225 6750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5238 6700 50  0001 C CNN
F 3 "" H 5200 6850 50  0000 C CNN
F 4 "25V" H 5200 6850 60  0001 C CNN "Voltage"
	1    5200 6850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR030
U 1 1 59913CCA
P 5200 7050
F 0 "#PWR030" H 5200 6800 50  0001 C CNN
F 1 "GND" H 5200 6900 50  0000 C CNN
F 2 "" H 5200 7050 50  0000 C CNN
F 3 "" H 5200 7050 50  0000 C CNN
	1    5200 7050
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR031
U 1 1 59915142
P 6100 4150
F 0 "#PWR031" H 6100 4000 50  0001 C CNN
F 1 "+12V" H 6100 4290 50  0000 C CNN
F 2 "" H 6100 4150 50  0000 C CNN
F 3 "" H 6100 4150 50  0000 C CNN
	1    6100 4150
	1    0    0    -1  
$EndComp
$Comp
L -12V #PWR44
U 1 1 59915221
P 5850 6400
F 0 "#PWR44" H 5850 6500 50  0001 C CNN
F 1 "-12V" H 5850 6550 50  0000 C CNN
F 2 "" H 5850 6400 50  0000 C CNN
F 3 "" H 5850 6400 50  0000 C CNN
	1    5850 6400
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 599157FD
P 5200 6150
F 0 "R19" V 5280 6150 50  0000 C CNN
F 1 "120k" V 5200 6150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5130 6150 50  0001 C CNN
F 3 "" H 5200 6150 50  0000 C CNN
	1    5200 6150
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 59916A5D
P 5000 6150
F 0 "C16" H 5025 6250 50  0000 L CNN
F 1 "56p" H 5025 6050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5038 6000 50  0001 C CNN
F 3 "" H 5000 6150 50  0000 C CNN
F 4 "25V" H 5000 6150 60  0001 C CNN "Voltage"
	1    5000 6150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR032
U 1 1 59917A59
P 4250 5600
F 0 "#PWR032" H 4250 5350 50  0001 C CNN
F 1 "GND" H 4250 5450 50  0000 C CNN
F 2 "" H 4250 5600 50  0000 C CNN
F 3 "" H 4250 5600 50  0000 C CNN
	1    4250 5600
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5992397D
P 2850 6050
F 0 "C9" H 2875 6150 50  0000 L CNN
F 1 "100n" H 2875 5950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2888 5900 50  0001 C CNN
F 3 "" H 2850 6050 50  0000 C CNN
F 4 "25V" H 2850 6050 60  0001 C CNN "Voltage"
	1    2850 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 599239EC
P 2850 6250
F 0 "#PWR033" H 2850 6000 50  0001 C CNN
F 1 "GND" H 2850 6100 50  0000 C CNN
F 2 "" H 2850 6250 50  0000 C CNN
F 3 "" H 2850 6250 50  0000 C CNN
	1    2850 6250
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 59926455
P 4600 4700
F 0 "C15" H 4625 4800 50  0000 L CNN
F 1 "56p" H 4625 4600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4638 4550 50  0001 C CNN
F 3 "" H 4600 4700 50  0000 C CNN
F 4 "25V" H 4600 4700 60  0001 C CNN "Voltage"
	1    4600 4700
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 599303BE
P 2550 5100
F 0 "C8" H 2575 5200 50  0000 L CNN
F 1 "4.7u" H 2575 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2588 4950 50  0001 C CNN
F 3 "" H 2550 5100 50  0000 C CNN
F 4 "25V" H 2550 5100 60  0001 C CNN "Voltage"
	1    2550 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 599305C3
P 2550 5300
F 0 "#PWR034" H 2550 5050 50  0001 C CNN
F 1 "GND" H 2550 5150 50  0000 C CNN
F 2 "" H 2550 5300 50  0000 C CNN
F 3 "" H 2550 5300 50  0000 C CNN
	1    2550 5300
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 59934628
P 5200 5550
F 0 "R18" V 5280 5550 50  0000 C CNN
F 1 "10k" V 5200 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5130 5550 50  0001 C CNN
F 3 "" H 5200 5550 50  0000 C CNN
	1    5200 5550
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 59934C4F
P 4550 5600
F 0 "C14" H 4575 5700 50  0000 L CNN
F 1 "100n" H 4575 5500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4588 5450 50  0001 C CNN
F 3 "" H 4550 5600 50  0000 C CNN
F 4 "25V" H 4550 5600 60  0001 C CNN "Voltage"
	1    4550 5600
	0    -1   -1   0   
$EndComp
$Comp
L +12V_ISO #PWR035
U 1 1 59AAFB5D
P 10250 4250
F 0 "#PWR035" H 10250 4100 50  0001 C CNN
F 1 "+12V_ISO" H 10250 4390 50  0000 C CNN
F 2 "" H 10250 4250 50  0000 C CNN
F 3 "" H 10250 4250 50  0000 C CNN
	1    10250 4250
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG036
U 1 1 59AB9D5F
P 2100 1100
F 0 "#FLG036" H 2100 1195 50  0001 C CNN
F 1 "PWR_FLAG" H 2100 1280 50  0000 C CNN
F 2 "" H 2100 1100 50  0000 C CNN
F 3 "" H 2100 1100 50  0000 C CNN
	1    2100 1100
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG037
U 1 1 59ABCF73
P 6100 4550
F 0 "#FLG037" H 6100 4645 50  0001 C CNN
F 1 "PWR_FLAG" H 6100 4730 50  0000 C CNN
F 2 "" H 6100 4550 50  0000 C CNN
F 3 "" H 6100 4550 50  0000 C CNN
	1    6100 4550
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG038
U 1 1 59ABD493
P 5850 6750
F 0 "#FLG038" H 5850 6845 50  0001 C CNN
F 1 "PWR_FLAG" H 5850 6930 50  0000 C CNN
F 2 "" H 5850 6750 50  0000 C CNN
F 3 "" H 5850 6750 50  0000 C CNN
	1    5850 6750
	-1   0    0    1   
$EndComp
Text Notes 8550 800  0    60   ~ 0
DC fan on +12V rail may introduce too much noise\nfor +12V current sensor!
$Comp
L C C22
U 1 1 59A9A501
P 7500 1400
F 0 "C22" H 7525 1500 50  0000 L CNN
F 1 "300p" H 7525 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7538 1250 50  0001 C CNN
F 3 "" H 7500 1400 50  0000 C CNN
F 4 "16V" H 7500 1400 60  0001 C CNN "Voltage"
	1    7500 1400
	1    0    0    -1  
$EndComp
Text Notes 6200 850  0    79   ~ 16
VREFA+ for ADC
$Comp
L +BATT #PWR039
U 1 1 59AC4ADB
P 2500 1050
F 0 "#PWR039" H 2500 900 50  0001 C CNN
F 1 "+BATT" H 2500 1190 50  0000 C CNN
F 2 "" H 2500 1050 50  0000 C CNN
F 3 "" H 2500 1050 50  0000 C CNN
	1    2500 1050
	1    0    0    -1  
$EndComp
$Comp
L MCP1501T-30E U10
U 1 1 59AFC908
P 6800 1300
F 0 "U10" H 6800 1000 60  0000 C CNN
F 1 "MCP1501T-30E" H 6800 1550 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 6750 1250 60  0001 C CNN
F 3 "" H 6750 1250 60  0000 C CNN
	1    6800 1300
	-1   0    0    -1  
$EndComp
$Comp
L +3VA #PWR040
U 1 1 59B05232
P 7500 1100
F 0 "#PWR040" H 7500 950 50  0001 C CNN
F 1 "+3VA" H 7500 1240 50  0000 C CNN
F 2 "" H 7500 1100 50  0000 C CNN
F 3 "" H 7500 1100 50  0000 C CNN
	1    7500 1100
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 59B0C97E
P 3100 1400
F 0 "C10" H 3125 1500 50  0000 L CNN
F 1 "10u" H 3125 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3138 1250 50  0001 C CNN
F 3 "" H 3100 1400 50  0000 C CNN
F 4 "16V" H 3100 1400 60  0001 C CNN "Voltage"
	1    3100 1400
	-1   0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 59B0FBE6
P 4150 1400
F 0 "C12" H 4175 1500 50  0000 L CNN
F 1 "10u" H 4175 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4188 1250 50  0001 C CNN
F 3 "" H 4150 1400 50  0000 C CNN
F 4 "16V" H 4150 1400 60  0001 C CNN "Voltage"
	1    4150 1400
	-1   0    0    -1  
$EndComp
$Comp
L +3V3 #PWR041
U 1 1 59B10F3D
P 4150 1050
F 0 "#PWR041" H 4150 900 50  0001 C CNN
F 1 "+3V3" H 4150 1190 50  0000 C CNN
F 2 "" H 4150 1050 50  0000 C CNN
F 3 "" H 4150 1050 50  0000 C CNN
	1    4150 1050
	1    0    0    -1  
$EndComp
Text Notes 3100 850  0    79   ~ 16
VDD, VDDA supply\n150 mA max
$Comp
L +3V3A #PWR042
U 1 1 59B14EC0
P 5350 950
F 0 "#PWR042" H 5350 800 50  0001 C CNN
F 1 "+3V3A" H 5350 1090 50  0000 C CNN
F 2 "" H 5350 950 50  0000 C CNN
F 3 "" H 5350 950 50  0000 C CNN
	1    5350 950 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR043
U 1 1 59B1D898
P 3600 1650
F 0 "#PWR043" H 3600 1400 50  0001 C CNN
F 1 "GND" H 3600 1500 50  0000 C CNN
F 2 "" H 3600 1650 50  0000 C CNN
F 3 "" H 3600 1650 50  0000 C CNN
	1    3600 1650
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 59B31F70
P 2350 5850
F 0 "R14" V 2430 5850 50  0000 C CNN
F 1 "10k" V 2350 5850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2280 5850 50  0001 C CNN
F 3 "" H 2350 5850 50  0000 C CNN
	1    2350 5850
	0    1    1    0   
$EndComp
$Comp
L +5V_ISO #PWR044
U 1 1 59B4088F
P 10250 4800
F 0 "#PWR044" H 10250 4650 50  0001 C CNN
F 1 "+5V_ISO" H 10250 4940 50  0000 C CNN
F 2 "" H 10250 4800 50  0000 C CNN
F 3 "" H 10250 4800 50  0000 C CNN
	1    10250 4800
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR045
U 1 1 59B428AC
P 10250 5250
F 0 "#PWR045" H 10250 5050 50  0001 C CNN
F 1 "GNDPWR" H 10250 5120 50  0000 C CNN
F 2 "" H 10250 5200 50  0000 C CNN
F 3 "" H 10250 5200 50  0000 C CNN
	1    10250 5250
	1    0    0    -1  
$EndComp
Text Notes 4650 4050 0    79   ~ 16
Fan, Iso, Sensor supply\n250 mA max (output)
Text Notes 6650 6500 0    79   ~ 16
Sensor supply\n60 mA max (output)
Text Notes 10150 4500 0    79   ~ 16
Optocoupler supply
Text Notes 10150 4000 0    79   ~ 16
Gate drive supply
$Comp
L LED-RESCUE-2kwMotorController D4
U 1 1 59B58EF7
P 4500 1400
F 0 "D4" H 4500 1500 50  0000 C CNN
F 1 "LED" H 4500 1300 50  0000 C CNN
F 2 "LEDs:LED_0603" H 4500 1400 50  0001 C CNN
F 3 "" H 4500 1400 50  0000 C CNN
	1    4500 1400
	0    -1   -1   0   
$EndComp
$Comp
L R R15
U 1 1 59B5AB06
P 4500 1800
F 0 "R15" V 4580 1800 50  0000 C CNN
F 1 "120" V 4500 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4430 1800 50  0001 C CNN
F 3 "" H 4500 1800 50  0000 C CNN
	1    4500 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR046
U 1 1 59B5AC15
P 4500 2000
F 0 "#PWR046" H 4500 1750 50  0001 C CNN
F 1 "GND" H 4500 1850 50  0000 C CNN
F 2 "" H 4500 2000 50  0000 C CNN
F 3 "" H 4500 2000 50  0000 C CNN
	1    4500 2000
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-2kwMotorController D5
U 1 1 59B74B7C
P 6450 4600
F 0 "D5" H 6450 4700 50  0000 C CNN
F 1 "LED" H 6450 4500 50  0000 C CNN
F 2 "LEDs:LED_0603" H 6450 4600 50  0001 C CNN
F 3 "" H 6450 4600 50  0000 C CNN
	1    6450 4600
	0    -1   -1   0   
$EndComp
$Comp
L R R20
U 1 1 59B74B82
P 6450 5000
F 0 "R20" V 6530 5000 50  0000 C CNN
F 1 "1.8k" V 6450 5000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6380 5000 50  0001 C CNN
F 3 "" H 6450 5000 50  0000 C CNN
	1    6450 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR047
U 1 1 59B74B89
P 6350 5450
F 0 "#PWR047" H 6350 5200 50  0001 C CNN
F 1 "GND" H 6350 5300 50  0000 C CNN
F 2 "" H 6350 5450 50  0000 C CNN
F 3 "" H 6350 5450 50  0000 C CNN
	1    6350 5450
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-2kwMotorController D6
U 1 1 59B7727F
P 6450 5900
F 0 "D6" H 6450 6000 50  0000 C CNN
F 1 "LED" H 6450 5800 50  0000 C CNN
F 2 "LEDs:LED_0603" H 6450 5900 50  0001 C CNN
F 3 "" H 6450 5900 50  0000 C CNN
	1    6450 5900
	0    -1   -1   0   
$EndComp
$Comp
L R R21
U 1 1 59B77285
P 6450 6300
F 0 "R21" V 6530 6300 50  0000 C CNN
F 1 "1.8k" V 6450 6300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6380 6300 50  0001 C CNN
F 3 "" H 6450 6300 50  0000 C CNN
	1    6450 6300
	1    0    0    -1  
$EndComp
$Comp
L C C26
U 1 1 59B42573
P 10250 5050
F 0 "C26" H 10275 5150 50  0000 L CNN
F 1 "1u" H 10275 4950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10288 4900 50  0001 C CNN
F 3 "" H 10250 5050 50  0000 C CNN
F 4 "25V" H 10250 5050 60  0001 C CNN "Voltage"
	1    10250 5050
	1    0    0    -1  
$EndComp
$Comp
L C C25
U 1 1 59BEDB92
P 9000 5000
F 0 "C25" H 9025 5100 50  0000 L CNN
F 1 "1u" H 9025 4900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9038 4850 50  0001 C CNN
F 3 "" H 9000 5000 50  0000 C CNN
F 4 "25V" H 9000 5000 60  0001 C CNN "Voltage"
	1    9000 5000
	1    0    0    -1  
$EndComp
$Comp
L MIC5225-5.0 U12
U 1 1 59BF0E85
P 9600 5000
F 0 "U12" H 9600 4750 60  0000 C CNN
F 1 "MIC5225-5.0" H 9650 5250 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 9600 5000 60  0001 C CNN
F 3 "" H 9600 5000 60  0000 C CNN
	1    9600 5000
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 59BF2D04
P 3800 3350
F 0 "C11" H 3825 3450 50  0000 L CNN
F 1 "1u" H 3825 3250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3838 3200 50  0001 C CNN
F 3 "" H 3800 3350 50  0000 C CNN
F 4 "16V" H 3800 3350 60  0001 C CNN "Voltage"
	1    3800 3350
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 59BF2D0A
P 2500 3300
F 0 "C7" H 2525 3400 50  0000 L CNN
F 1 "1u" H 2525 3200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2538 3150 50  0001 C CNN
F 3 "" H 2500 3300 50  0000 C CNN
F 4 "16V" H 2500 3300 60  0001 C CNN "Voltage"
	1    2500 3300
	1    0    0    -1  
$EndComp
$Comp
L MIC5225-5.0 U7
U 1 1 59BF2D11
P 3150 3300
F 0 "U7" H 3150 3050 60  0000 C CNN
F 1 "MIC5225-5.0" H 3200 3550 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 3150 3300 60  0001 C CNN
F 3 "" H 3150 3300 60  0000 C CNN
	1    3150 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR048
U 1 1 59BF3490
P 2500 3500
F 0 "#PWR048" H 2500 3250 50  0001 C CNN
F 1 "GND" H 2500 3350 50  0000 C CNN
F 2 "" H 2500 3500 50  0000 C CNN
F 3 "" H 2500 3500 50  0000 C CNN
	1    2500 3500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR049
U 1 1 59BF5180
P 3800 3100
F 0 "#PWR049" H 3800 2950 50  0001 C CNN
F 1 "+5V" H 3800 3240 50  0000 C CNN
F 2 "" H 3800 3100 50  0000 C CNN
F 3 "" H 3800 3100 50  0000 C CNN
	1    3800 3100
	1    0    0    -1  
$EndComp
Text Notes 3950 3000 0    79   ~ 16
Encoder supply\n80 mA max
$Comp
L GNDCBAT #PWR050
U 1 1 59C0C795
P 1250 7300
F 0 "#PWR050" H 1250 7050 50  0001 C CNN
F 1 "GNDCBAT" H 1250 7150 50  0000 C CNN
F 2 "" H 1250 7300 50  0000 C CNN
F 3 "" H 1250 7300 50  0000 C CNN
	1    1250 7300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR051
U 1 1 59C60D75
P 9900 2900
F 0 "#PWR051" H 9900 2650 50  0001 C CNN
F 1 "GND" H 9900 2750 50  0000 C CNN
F 2 "" H 9900 2900 50  0000 C CNN
F 3 "" H 9900 2900 50  0000 C CNN
	1    9900 2900
	1    0    0    -1  
$EndComp
$Comp
L D D8
U 1 1 59C60D82
P 9900 2050
F 0 "D8" H 9900 2150 50  0000 C CNN
F 1 "SK310-B" H 9900 1950 50  0000 C CNN
F 2 "2kwMotorController:DIODE_POWERMITE" H 9900 2050 50  0001 C CNN
F 3 "" H 9900 2050 50  0000 C CNN
	1    9900 2050
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR052
U 1 1 59C60D8A
P 9900 1800
F 0 "#PWR052" H 9900 1650 50  0001 C CNN
F 1 "+12V" H 9900 1940 50  0000 C CNN
F 2 "" H 9900 1800 50  0000 C CNN
F 3 "" H 9900 1800 50  0000 C CNN
	1    9900 1800
	1    0    0    -1  
$EndComp
Text HLabel 7850 2700 0    60   Input ~ 0
PWM_FAN
Text Notes 9000 1525 0    79   ~ 16
12V DC FAN
$Comp
L C C27
U 1 1 59C62513
P 10650 2250
F 0 "C27" H 10675 2350 50  0000 L CNN
F 1 "100n" H 10675 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10688 2100 50  0001 C CNN
F 3 "" H 10650 2250 50  0000 C CNN
F 4 "25V" H 10650 2250 60  0001 C CNN "Voltage"
	1    10650 2250
	1    0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 59C8C646
P 9350 2500
F 0 "R24" V 9430 2500 50  0000 C CNN
F 1 "180" V 9350 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9280 2500 50  0001 C CNN
F 3 "" H 9350 2500 50  0000 C CNN
	1    9350 2500
	0    1    1    0   
$EndComp
$Comp
L R R25
U 1 1 59CA88AD
P 8350 2900
F 0 "R25" V 8430 2900 50  0000 C CNN
F 1 "10k" V 8350 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8280 2900 50  0001 C CNN
F 3 "" H 8350 2900 50  0000 C CNN
	1    8350 2900
	-1   0    0    1   
$EndComp
$Comp
L C C28
U 1 1 59BA005F
P 10900 2250
F 0 "C28" H 10925 2350 50  0000 L CNN
F 1 "1u" H 10925 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10938 2100 50  0001 C CNN
F 3 "" H 10900 2250 50  0000 C CNN
F 4 "25V" H 10900 2250 60  0001 C CNN "Voltage"
	1    10900 2250
	1    0    0    -1  
$EndComp
$Comp
L NMOS_8PIN Q2
U 1 1 59BD8B92
P 9800 2500
F 0 "Q2" H 10100 2550 50  0000 R CNN
F 1 "DMG4712SSS" H 10450 2450 50  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 10000 2600 50  0001 C CNN
F 3 "" H 9800 2500 50  0000 C CNN
	1    9800 2500
	1    0    0    -1  
$EndComp
$Comp
L NMOS_8PIN Q1
U 1 1 59BDA6D3
P 2100 7150
F 0 "Q1" V 2100 7400 50  0000 R CNN
F 1 "DMG4712SSS" V 2350 7350 50  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2300 7250 50  0001 C CNN
F 3 "" H 2100 7150 50  0000 C CNN
	1    2100 7150
	0    -1   1    0   
$EndComp
$Comp
L C C23
U 1 1 59BA3720
P 7200 4550
F 0 "C23" H 7225 4650 50  0000 L CNN
F 1 "2.2u" H 7225 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7238 4400 50  0001 C CNN
F 3 "" H 7200 4550 50  0000 C CNN
F 4 "25V" H 7200 4550 60  0001 C CNN "Voltage"
	1    7200 4550
	1    0    0    -1  
$EndComp
$Comp
L C C24
U 1 1 59BA475B
P 8350 4550
F 0 "C24" H 8375 4650 50  0000 L CNN
F 1 "2.2u" H 8375 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8388 4400 50  0001 C CNN
F 3 "" H 8350 4550 50  0000 C CNN
F 4 "25V" H 8350 4550 60  0001 C CNN "Voltage"
	1    8350 4550
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 59BA7A13
P 8550 4950
F 0 "R22" V 8630 4950 50  0000 C CNN
F 1 "2.2k" V 8550 4950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8480 4950 50  0001 C CNN
F 3 "" H 8550 4950 50  0000 C CNN
	1    8550 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR053
U 1 1 59BAC5F2
P 3800 3550
F 0 "#PWR053" H 3800 3300 50  0001 C CNN
F 1 "GND" H 3800 3400 50  0000 C CNN
F 2 "" H 3800 3550 50  0000 C CNN
F 3 "" H 3800 3550 50  0000 C CNN
	1    3800 3550
	1    0    0    -1  
$EndComp
$Comp
L C C19
U 1 1 59BD5A07
P 5450 6850
F 0 "C19" H 5475 6950 50  0000 L CNN
F 1 "1u" H 5475 6750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5488 6700 50  0001 C CNN
F 3 "" H 5450 6850 50  0000 C CNN
F 4 "25V" H 5450 6850 60  0001 C CNN "Voltage"
	1    5450 6850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR054
U 1 1 59BD5BF7
P 5450 7050
F 0 "#PWR054" H 5450 6800 50  0001 C CNN
F 1 "GND" H 5450 6900 50  0000 C CNN
F 2 "" H 5450 7050 50  0000 C CNN
F 3 "" H 5450 7050 50  0000 C CNN
	1    5450 7050
	1    0    0    -1  
$EndComp
$Comp
L C C21
U 1 1 59BD796B
P 5700 4550
F 0 "C21" H 5725 4650 50  0000 L CNN
F 1 "1u" H 5725 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5738 4400 50  0001 C CNN
F 3 "" H 5700 4550 50  0000 C CNN
F 4 "25V" H 5700 4550 60  0001 C CNN "Voltage"
	1    5700 4550
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L3
U 1 1 59BDEC2A
P 4850 6550
F 0 "L3" H 4750 6650 50  0000 C CNN
F 1 "15u" H 4950 6650 50  0000 C CNN
F 2 "2kwMotorController:INDUCTOR_NR40xx" H 4850 6550 50  0001 C CNN
F 3 "NR4012T150M" H 4850 6500 50  0000 C CNN
	1    4850 6550
	1    0    0    1   
$EndComp
$Comp
L R R56
U 1 1 59BF0670
P 5100 1150
F 0 "R56" V 5180 1150 50  0000 C CNN
F 1 "1" V 5100 1150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5030 1150 50  0001 C CNN
F 3 "" H 5100 1150 50  0000 C CNN
	1    5100 1150
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG055
U 1 1 59C7F9B5
P 5750 1000
F 0 "#FLG055" H 5750 1095 50  0001 C CNN
F 1 "PWR_FLAG" H 5750 1180 50  0000 C CNN
F 2 "" H 5750 1000 50  0000 C CNN
F 3 "" H 5750 1000 50  0000 C CNN
	1    5750 1000
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG056
U 1 1 59C81E35
P 1450 7150
F 0 "#FLG056" H 1450 7245 50  0001 C CNN
F 1 "PWR_FLAG" H 1450 7330 50  0000 C CNN
F 2 "" H 1450 7150 50  0000 C CNN
F 3 "" H 1450 7150 50  0000 C CNN
	1    1450 7150
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG057
U 1 1 59C83C70
P 7900 1550
F 0 "#FLG057" H 7900 1645 50  0001 C CNN
F 1 "PWR_FLAG" H 7900 1730 50  0000 C CNN
F 2 "" H 7900 1550 50  0000 C CNN
F 3 "" H 7900 1550 50  0000 C CNN
	1    7900 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C20
U 1 1 59D2AF3F
P 6100 1300
F 0 "C20" H 6110 1370 50  0000 L CNN
F 1 "2.2u" H 6110 1220 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6100 1300 50  0001 C CNN
F 3 "" H 6100 1300 50  0000 C CNN
F 4 "16V" H 6100 1300 60  0001 C CNN "Voltage"
	1    6100 1300
	1    0    0    -1  
$EndComp
Text Label 4200 5400 0    60   ~ 0
SMPS_VREF
Text Label 4200 5050 0    60   ~ 0
SMPS_FB1N
Text Label 5150 5800 2    60   ~ 0
SMPS_FB2P
Text Label 4150 4400 3    60   ~ 0
SMPS_SW1
Text Label 4150 5950 3    60   ~ 0
SMPS_SW2
Text Label 2800 5700 2    60   ~ 0
SMPS_SHDN
$Comp
L R-78E3.3-0.5 U9
U 1 1 59D01002
P 3600 1150
F 0 "U9" H 3350 950 60  0000 C CNN
F 1 "R-78E3.3-0.5" H 3600 1350 60  0000 C CNN
F 2 "2kwMotorController:R-78E-0.5" H 3600 1050 60  0001 C CNN
F 3 "" H 3600 1050 60  0000 C CNN
	1    3600 1150
	1    0    0    -1  
$EndComp
$Comp
L C C72
U 1 1 59D0EA2D
P 5350 1400
F 0 "C72" H 5375 1500 50  0000 L CNN
F 1 "10u" H 5375 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5388 1250 50  0001 C CNN
F 3 "" H 5350 1400 50  0000 C CNN
F 4 "16V" H 5350 1400 60  0001 C CNN "Voltage"
	1    5350 1400
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR058
U 1 1 59D11411
P 5350 1600
F 0 "#PWR058" H 5350 1350 50  0001 C CNN
F 1 "GND" H 5350 1450 50  0000 C CNN
F 2 "" H 5350 1600 50  0000 C CNN
F 3 "" H 5350 1600 50  0000 C CNN
	1    5350 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR059
U 1 1 59D11A7E
P 6100 1500
F 0 "#PWR059" H 6100 1250 50  0001 C CNN
F 1 "GND" H 6100 1350 50  0000 C CNN
F 2 "" H 6100 1500 50  0000 C CNN
F 3 "" H 6100 1500 50  0000 C CNN
	1    6100 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR060
U 1 1 59D11B4C
P 7500 1650
F 0 "#PWR060" H 7500 1400 50  0001 C CNN
F 1 "GND" H 7500 1500 50  0000 C CNN
F 2 "" H 7500 1650 50  0000 C CNN
F 3 "" H 7500 1650 50  0000 C CNN
	1    7500 1650
	1    0    0    -1  
$EndComp
Text Label 2600 1150 0    60   ~ 0
CTRL_BATT
$Comp
L 74LVC1G17 U21
U 1 1 59CB1F1E
P 8750 2500
F 0 "U21" H 8750 2200 60  0000 C CNN
F 1 "74LVC1G17" H 8750 2850 60  0000 C CNN
F 2 "2kwMotorController:SC-74A" H 8750 2200 60  0001 C CNN
F 3 "" H 8750 2200 60  0000 C CNN
	1    8750 2500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C82
U 1 1 59CB357A
P 8350 2400
F 0 "C82" H 8360 2470 50  0000 L CNN
F 1 "100n" H 8400 2350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8350 2400 50  0001 C CNN
F 3 "" H 8350 2400 50  0000 C CNN
F 4 "25V" H 8350 2400 60  0001 C CNN "Voltage"
	1    8350 2400
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR061
U 1 1 59CB3CCD
P 8350 2250
F 0 "#PWR061" H 8350 2100 50  0001 C CNN
F 1 "+5V" H 8350 2390 50  0000 C CNN
F 2 "" H 8350 2250 50  0000 C CNN
F 3 "" H 8350 2250 50  0000 C CNN
	1    8350 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR062
U 1 1 59CB43C5
P 8350 2550
F 0 "#PWR062" H 8350 2300 50  0001 C CNN
F 1 "GND" H 8250 2550 50  0000 C CNN
F 2 "" H 8350 2550 50  0000 C CNN
F 3 "" H 8350 2550 50  0000 C CNN
	1    8350 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR063
U 1 1 59CB9A69
P 8350 3100
F 0 "#PWR063" H 8350 2850 50  0001 C CNN
F 1 "GND" H 8250 3100 50  0000 C CNN
F 2 "" H 8350 3100 50  0000 C CNN
F 3 "" H 8350 3100 50  0000 C CNN
	1    8350 3100
	1    0    0    -1  
$EndComp
Text Label 9500 2400 2    60   ~ 0
FAN_GATE
Text Label 10050 2250 0    60   ~ 0
FAN_DRAIN
$Comp
L TEST_POINT TP14
U 1 1 59C85194
P 6450 4300
F 0 "TP14" H 6450 4500 60  0000 C CNN
F 1 "TEST_POINT" H 6450 4700 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 6450 4300 60  0001 C CNN
F 3 "" H 6450 4300 60  0000 C CNN
	1    6450 4300
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP13
U 1 1 59C85BD5
P 6150 6500
F 0 "TP13" H 6150 6700 60  0000 C CNN
F 1 "TEST_POINT" H 6150 6900 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 6150 6500 60  0001 C CNN
F 3 "" H 6150 6500 60  0000 C CNN
	1    6150 6500
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP16
U 1 1 59C8644C
P 9150 4250
F 0 "TP16" H 9150 4450 60  0000 C CNN
F 1 "TEST_POINT" H 9150 4650 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 9150 4250 60  0001 C CNN
F 3 "" H 9150 4250 60  0000 C CNN
	1    9150 4250
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP18
U 1 1 59C8652D
P 10600 4850
F 0 "TP18" H 10600 5050 60  0000 C CNN
F 1 "TEST_POINT" H 10600 5250 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 10600 4850 60  0001 C CNN
F 3 "" H 10600 4850 60  0000 C CNN
	1    10600 4850
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP17
U 1 1 59C88596
P 9550 2600
F 0 "TP17" H 9550 2800 60  0000 C CNN
F 1 "TEST_POINT" H 9550 3000 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 9550 2600 60  0001 C CNN
F 3 "" H 9550 2600 60  0000 C CNN
	1    9550 2600
	-1   0    0    1   
$EndComp
$Comp
L TEST_POINT TP15
U 1 1 59C8877A
P 7900 2600
F 0 "TP15" H 7900 2800 60  0000 C CNN
F 1 "TEST_POINT" H 7900 3000 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 7900 2600 60  0001 C CNN
F 3 "" H 7900 2600 60  0000 C CNN
	1    7900 2600
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP12
U 1 1 59C89527
P 6100 1100
F 0 "TP12" H 6100 1300 60  0000 C CNN
F 1 "TEST_POINT" H 6100 1500 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 6100 1100 60  0001 C CNN
F 3 "" H 6100 1100 60  0000 C CNN
	1    6100 1100
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP11
U 1 1 59C8ADE3
P 4500 1100
F 0 "TP11" H 4500 1300 60  0000 C CNN
F 1 "TEST_POINT" H 4500 1500 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 4500 1100 60  0001 C CNN
F 3 "" H 4500 1100 60  0000 C CNN
	1    4500 1100
	1    0    0    -1  
$EndComp
$Comp
L TEST_POINT TP9
U 1 1 59C8BC84
P 2500 1250
F 0 "TP9" H 2500 1450 60  0000 C CNN
F 1 "TEST_POINT" H 2500 1650 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 2500 1250 60  0001 C CNN
F 3 "" H 2500 1250 60  0000 C CNN
	1    2500 1250
	-1   0    0    1   
$EndComp
$Comp
L TEST_POINT TP10
U 1 1 59C8C708
P 3650 3100
F 0 "TP10" H 3650 3300 60  0000 C CNN
F 1 "TEST_POINT" H 3650 3500 60  0001 C CNN
F 2 "2kwMotorController:TEST_PAD_1.27mm" H 3650 3100 60  0001 C CNN
F 3 "" H 3650 3100 60  0000 C CNN
	1    3650 3100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P2
U 1 1 5A0A5045
P 9400 2050
F 0 "P2" H 9400 2300 50  0000 C CNN
F 1 "CONN_01X04" V 9500 2050 50  0000 C CNN
F 2 "2kwMotorController:TERM_4P_2-4_1990025" H 9400 2050 50  0001 C CNN
F 3 "" H 9400 2050 50  0000 C CNN
	1    9400 2050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1150 1250 1250 1250
Wire Wire Line
	2500 1050 2500 1250
Wire Wire Line
	8150 4300 10250 4300
Connection ~ 9150 4300
Wire Wire Line
	8150 4500 8200 4500
Wire Wire Line
	8200 4500 8200 5200
Wire Wire Line
	8750 4750 8750 4800
Wire Wire Line
	8750 5150 8750 5100
Wire Wire Line
	8200 5150 9200 5150
Connection ~ 8200 5150
Wire Wire Line
	4550 4350 7500 4350
Wire Wire Line
	7500 4450 7400 4450
Wire Wire Line
	7400 4450 7400 5150
Wire Wire Line
	8750 4350 8750 4300
Connection ~ 8750 4300
Wire Wire Line
	1250 1250 1250 7300
Wire Wire Line
	1250 7250 1900 7250
Wire Wire Line
	2100 1100 2100 6600
Wire Wire Line
	2300 7250 2600 7250
Wire Notes Line
	7900 4650 7900 5750
Wire Notes Line
	7750 4650 7750 5750
Wire Wire Line
	4000 4350 4250 4350
Wire Wire Line
	4150 4350 4150 4900
Connection ~ 4150 4350
Wire Wire Line
	4150 4900 4100 4900
Wire Wire Line
	4100 5050 4950 5050
Wire Wire Line
	4850 4850 4850 5050
Connection ~ 4850 5050
Wire Wire Line
	5250 5050 5700 5050
Wire Wire Line
	5450 4700 5450 5100
Wire Wire Line
	4850 4350 4850 4550
Connection ~ 5450 5050
Connection ~ 5450 4350
Wire Wire Line
	4100 5900 4150 5900
Wire Wire Line
	4150 5900 4150 6550
Wire Wire Line
	4100 6550 4200 6550
Connection ~ 4150 6550
Wire Wire Line
	4550 7050 4550 7000
Wire Wire Line
	4500 6550 4600 6550
Wire Wire Line
	4550 6550 4550 6700
Connection ~ 4550 6550
Wire Wire Line
	5100 6550 6650 6550
Wire Wire Line
	5200 6300 5200 6700
Wire Wire Line
	5200 7050 5200 7000
Connection ~ 5200 6550
Wire Wire Line
	6100 4150 6100 4550
Wire Wire Line
	5200 5700 5200 6000
Wire Wire Line
	4100 5600 4400 5600
Wire Wire Line
	2850 6250 2850 6200
Wire Wire Line
	2500 5850 2900 5850
Wire Wire Line
	2550 5250 2550 5300
Wire Wire Line
	2100 6550 3600 6550
Wire Wire Line
	2100 4350 3500 4350
Wire Wire Line
	4100 5200 4150 5200
Wire Wire Line
	4150 5200 4150 5400
Wire Wire Line
	4100 5400 5200 5400
Connection ~ 4150 5400
Wire Wire Line
	4750 5600 4700 5600
Wire Wire Line
	4750 5400 4750 5600
Connection ~ 4750 5400
Connection ~ 4250 5600
Wire Wire Line
	10250 4300 10250 4250
Wire Wire Line
	7250 1450 7300 1450
Wire Wire Line
	6350 1300 6300 1300
Wire Wire Line
	7250 1300 7300 1300
Wire Wire Line
	7300 1300 7300 1600
Connection ~ 6100 1150
Wire Wire Line
	7500 1550 7500 1650
Wire Wire Line
	6300 1300 6300 1150
Connection ~ 6300 1150
Wire Wire Line
	3100 1550 3100 1600
Wire Wire Line
	3100 1600 4150 1600
Wire Wire Line
	6100 1450 6350 1450
Wire Wire Line
	4150 1050 4150 1250
Connection ~ 3600 1600
Wire Wire Line
	5250 1150 6350 1150
Wire Wire Line
	3600 1500 3600 1650
Connection ~ 2500 1150
Connection ~ 2100 4350
Connection ~ 6100 4350
Wire Wire Line
	2850 5550 2850 5900
Connection ~ 2850 5850
Wire Wire Line
	2900 5250 2550 5250
Wire Wire Line
	2100 4950 2900 4950
Connection ~ 2100 4950
Connection ~ 2550 4950
Wire Wire Line
	2900 5550 2850 5550
Wire Wire Line
	5850 6400 5850 6750
Connection ~ 5850 6550
Wire Wire Line
	5450 4400 5450 4350
Wire Wire Line
	9150 4250 9150 5000
Wire Wire Line
	9000 4850 9200 4850
Wire Wire Line
	9150 5000 9200 5000
Connection ~ 9150 4850
Connection ~ 8750 5150
Wire Wire Line
	10250 4800 10250 4900
Wire Wire Line
	10050 4850 10600 4850
Wire Wire Line
	10250 5250 10250 5200
Connection ~ 10250 4850
Wire Wire Line
	4500 1100 4500 1200
Wire Wire Line
	4500 1650 4500 1600
Wire Wire Line
	4500 2000 4500 1950
Wire Wire Line
	6450 4850 6450 4800
Wire Wire Line
	6450 5150 6450 5700
Wire Wire Line
	6450 4300 6450 4400
Connection ~ 6450 4350
Wire Wire Line
	6450 6150 6450 6100
Wire Wire Line
	6450 6450 6450 6550
Connection ~ 6450 6550
Wire Wire Line
	6350 5450 6350 5400
Wire Wire Line
	6350 5400 6450 5400
Connection ~ 6450 5400
Wire Wire Line
	5000 6000 5000 5950
Wire Wire Line
	5000 5950 5200 5950
Connection ~ 5200 5950
Wire Wire Line
	5000 6300 5000 6350
Wire Wire Line
	5000 6350 5200 6350
Connection ~ 5200 6350
Wire Wire Line
	4600 4550 4600 4500
Wire Wire Line
	4600 4500 4850 4500
Connection ~ 4850 4500
Wire Wire Line
	4600 4850 4600 4900
Wire Wire Line
	4600 4900 4850 4900
Connection ~ 4850 4900
Connection ~ 4850 4350
Connection ~ 9000 5150
Wire Wire Line
	2700 3300 2750 3300
Connection ~ 2700 3150
Wire Wire Line
	3800 3100 3800 3200
Wire Wire Line
	3600 3150 3800 3150
Wire Wire Line
	3800 3550 3800 3500
Connection ~ 3800 3150
Wire Wire Line
	2500 3500 2500 3450
Wire Wire Line
	2100 3150 2750 3150
Connection ~ 2100 3150
Connection ~ 2500 3150
Wire Wire Line
	2700 3300 2700 3150
Wire Wire Line
	2500 3450 2750 3450
Connection ~ 1250 7250
Wire Wire Line
	9900 2700 9900 2900
Wire Wire Line
	9900 2200 9900 2300
Wire Wire Line
	9900 1800 9900 1900
Connection ~ 9900 1850
Connection ~ 9900 2250
Wire Wire Line
	10650 1850 10650 2100
Wire Wire Line
	10650 2850 10650 2400
Wire Wire Line
	9900 2850 10900 2850
Connection ~ 9900 2850
Wire Wire Line
	9500 2500 9600 2500
Wire Wire Line
	10900 1850 10900 2100
Connection ~ 10650 1850
Wire Wire Line
	10900 2850 10900 2400
Connection ~ 10650 2850
Wire Wire Line
	7200 4400 7200 4350
Connection ~ 7200 4350
Wire Wire Line
	7200 4700 7200 4750
Wire Wire Line
	7200 4750 7400 4750
Connection ~ 7400 4750
Wire Wire Line
	8350 4400 8350 4300
Connection ~ 8350 4300
Wire Wire Line
	8350 4700 8350 4750
Wire Wire Line
	8350 4750 8200 4750
Connection ~ 8200 4750
Wire Wire Line
	8550 4800 8550 4300
Connection ~ 8550 4300
Wire Wire Line
	8550 5100 8550 5150
Connection ~ 8550 5150
Wire Wire Line
	2100 6900 2100 6950
Wire Wire Line
	5450 7050 5450 7000
Wire Wire Line
	5450 6550 5450 6700
Connection ~ 5450 6550
Wire Wire Line
	5700 5050 5700 4700
Wire Wire Line
	5700 4400 5700 4350
Connection ~ 5700 4350
Connection ~ 2100 1150
Wire Wire Line
	5350 1550 5350 1600
Wire Wire Line
	5350 950  5350 1250
Wire Wire Line
	2100 5850 2200 5850
Connection ~ 2100 5850
Connection ~ 2100 6550
Wire Wire Line
	7300 1600 7900 1600
Connection ~ 7500 1600
Connection ~ 7300 1450
Wire Wire Line
	6100 1400 6100 1500
Wire Wire Line
	1450 7150 1450 7250
Connection ~ 1450 7250
Wire Wire Line
	7900 1600 7900 1550
Connection ~ 5350 1150
Connection ~ 6100 1450
Wire Wire Line
	6100 1100 6100 1200
Wire Wire Line
	5750 1000 5750 1150
Connection ~ 5750 1150
Connection ~ 4500 1150
Connection ~ 6150 6550
Wire Wire Line
	3650 3100 3650 3150
Connection ~ 3650 3150
Wire Wire Line
	2850 5850 2800 5700
Wire Wire Line
	4100 5750 4150 5750
Wire Wire Line
	4150 5750 4150 5800
Wire Wire Line
	4150 5800 5200 5800
Connection ~ 5200 5800
Wire Wire Line
	3100 1150 3100 1250
Wire Wire Line
	4150 1600 4150 1550
Connection ~ 4150 1150
Connection ~ 3100 1150
Wire Wire Line
	8350 2300 8400 2300
Wire Wire Line
	8350 2250 8350 2300
Connection ~ 8350 2500
Wire Wire Line
	9150 2500 9200 2500
Wire Wire Line
	8350 2500 8350 2550
Wire Wire Line
	8350 2500 8400 2500
Wire Wire Line
	7850 2700 8400 2700
Wire Wire Line
	8350 2750 8350 2700
Connection ~ 8350 2700
Wire Wire Line
	8350 3050 8350 3100
Wire Wire Line
	7900 2600 7900 2700
Connection ~ 7900 2700
Wire Wire Line
	9500 2400 9550 2500
Wire Wire Line
	9550 2500 9550 2600
Connection ~ 9550 2500
Wire Wire Line
	6150 6500 6150 6550
Wire Wire Line
	2600 7250 2600 7300
Wire Wire Line
	4150 1150 4950 1150
Wire Wire Line
	1150 1150 3100 1150
Wire Wire Line
	7500 1250 7500 1100
Wire Wire Line
	7250 1150 7500 1150
Connection ~ 7500 1150
Connection ~ 9700 1850
Wire Wire Line
	9650 1850 10900 1850
Wire Wire Line
	9600 1900 9650 1900
Wire Wire Line
	9650 2250 10050 2250
Wire Wire Line
	9600 2100 9700 2100
Wire Wire Line
	9600 2200 9650 2200
Wire Wire Line
	9650 2000 9650 2250
Wire Wire Line
	9700 2100 9700 1850
Wire Wire Line
	9600 2000 9650 2000
Connection ~ 9650 2200
Wire Wire Line
	9650 1900 9650 1850
$EndSCHEMATC
