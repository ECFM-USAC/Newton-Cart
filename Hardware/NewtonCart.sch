EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:ESP32-WROOM-32 U?
U 1 1 60753017
P 2650 4650
F 0 "U?" H 2650 6231 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 2650 6140 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 2650 3150 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 2350 4700 50  0001 C CNN
	1    2650 4650
	1    0    0    -1  
$EndComp
Text GLabel 1450 5050 0    50   Input ~ 0
MPU_SCL
Text GLabel 1450 4650 0    50   Input ~ 0
MPU_SDA
Wire Wire Line
	2050 4650 1450 4650
Wire Wire Line
	2050 5050 1450 5050
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 607591AA
P 1050 1250
F 0 "J?" H 942 1535 50  0000 C CNN
F 1 "Battery Pins" H 942 1444 50  0000 C CNN
F 2 "" H 1050 1250 50  0001 C CNN
F 3 "~" H 1050 1250 50  0001 C CNN
	1    1050 1250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1250 1150 1550 1150
Wire Wire Line
	1250 1250 1550 1250
Wire Wire Line
	1550 1250 1550 1350
Wire Wire Line
	1250 1350 1550 1350
Connection ~ 1550 1350
$Comp
L Regulator_Linear:LM1117-3.3 U?
U 1 1 6075D169
P 4000 1100
F 0 "U?" H 4000 1342 50  0000 C CNN
F 1 "LM1117-3.3" H 4000 1251 50  0000 C CNN
F 2 "" H 4000 1100 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 4000 1100 50  0001 C CNN
	1    4000 1100
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 6075E5C6
P 4000 1650
F 0 "#PWR?" H 4000 1400 50  0001 C CNN
F 1 "Earth" H 4000 1500 50  0001 C CNN
F 2 "" H 4000 1650 50  0001 C CNN
F 3 "~" H 4000 1650 50  0001 C CNN
	1    4000 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1100 3600 1100
Wire Wire Line
	4000 1400 4000 1550
$Comp
L Device:CP1_Small C?
U 1 1 6076062A
P 3600 1400
F 0 "C?" H 3691 1446 50  0000 L CNN
F 1 "CP1_Small" H 3691 1355 50  0000 L CNN
F 2 "" H 3600 1400 50  0001 C CNN
F 3 "~" H 3600 1400 50  0001 C CNN
	1    3600 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 60760C72
P 4400 1400
F 0 "C?" H 4491 1446 50  0000 L CNN
F 1 "CP1_Small" H 4491 1355 50  0000 L CNN
F 2 "" H 4400 1400 50  0001 C CNN
F 3 "~" H 4400 1400 50  0001 C CNN
	1    4400 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1100 3600 1300
Connection ~ 3600 1100
Wire Wire Line
	3600 1100 3700 1100
Wire Wire Line
	3600 1500 3600 1550
Wire Wire Line
	3600 1550 4000 1550
Connection ~ 4000 1550
Wire Wire Line
	4000 1550 4000 1650
Wire Wire Line
	4300 1100 4400 1100
Wire Wire Line
	4400 1100 4400 1300
Wire Wire Line
	4400 1500 4400 1550
Wire Wire Line
	4400 1550 4000 1550
Text GLabel 4900 1100 2    50   Input ~ 0
3.3Reg
Wire Wire Line
	4900 1100 4400 1100
Connection ~ 4400 1100
$Comp
L power:Earth #PWR?
U 1 1 607648AA
P 2650 6250
F 0 "#PWR?" H 2650 6000 50  0001 C CNN
F 1 "Earth" H 2650 6100 50  0001 C CNN
F 2 "" H 2650 6250 50  0001 C CNN
F 3 "~" H 2650 6250 50  0001 C CNN
	1    2650 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 6050 2650 6250
Text GLabel 2800 3000 2    50   Input ~ 0
3.3Reg
Wire Wire Line
	2800 3000 2650 3000
Wire Wire Line
	2650 3000 2650 3250
Text GLabel 3900 4650 2    50   Input ~ 0
LC_SCK
Text GLabel 3900 4800 2    50   Input ~ 0
LC_DT
Wire Wire Line
	3250 4650 3900 4650
Wire Wire Line
	3900 4800 3650 4800
Wire Wire Line
	3650 4800 3650 4750
Wire Wire Line
	3650 4750 3250 4750
Text GLabel 3900 3650 2    50   Input ~ 0
US_TRIGGER
Text GLabel 3900 3850 2    50   Input ~ 0
US_ECHO
Wire Wire Line
	3250 3650 3900 3650
Wire Wire Line
	3250 3850 3900 3850
Text GLabel 3900 4000 2    50   Input ~ 0
VEL_PULSE
Wire Wire Line
	3900 4000 3800 4000
Wire Wire Line
	3800 4000 3800 3950
Wire Wire Line
	3800 3950 3250 3950
$Comp
L power:+3V8 #PWR?
U 1 1 60887519
P 1550 800
F 0 "#PWR?" H 1550 650 50  0001 C CNN
F 1 "+3V8" H 1565 973 50  0000 C CNN
F 2 "" H 1550 800 50  0001 C CNN
F 3 "" H 1550 800 50  0001 C CNN
	1    1550 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 800  1550 950 
$Comp
L Connector:Conn_01x06_Female J?
U 1 1 60888CF7
P 2650 1150
F 0 "J?" H 2678 1126 50  0000 L CNN
F 1 "TP4056 Module" H 2678 1035 50  0000 L CNN
F 2 "" H 2650 1150 50  0001 C CNN
F 3 "~" H 2650 1150 50  0001 C CNN
	1    2650 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 950  1550 950 
Connection ~ 1550 950 
Wire Wire Line
	1550 950  1550 1150
Wire Wire Line
	2450 1050 1750 1050
Wire Wire Line
	1750 1050 1750 1350
Wire Wire Line
	1750 1350 1550 1350
Text GLabel 3150 650  0    50   Input ~ 0
4056OUT+
Wire Wire Line
	3150 650  3350 650 
Wire Wire Line
	3350 650  3350 1100
Wire Wire Line
	2300 1150 2450 1150
Text GLabel 2300 1150 0    50   Input ~ 0
4056OUT+
Text GLabel 2300 1350 0    50   Input ~ 0
4056OUT-
Text GLabel 2300 1250 0    50   Input ~ 0
USB+
Text GLabel 2300 1450 0    50   Input ~ 0
USB-
Text GLabel 1400 1650 0    50   Input ~ 0
Battery(-)
Wire Wire Line
	1400 1650 1550 1650
Wire Wire Line
	1550 1350 1550 1650
$Comp
L power:Earth #PWR?
U 1 1 60896943
P 2400 1600
F 0 "#PWR?" H 2400 1350 50  0001 C CNN
F 1 "Earth" H 2400 1450 50  0001 C CNN
F 2 "" H 2400 1600 50  0001 C CNN
F 3 "~" H 2400 1600 50  0001 C CNN
	1    2400 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 60897D54
P 6350 1050
F 0 "J?" H 6378 1026 50  0000 L CNN
F 1 "BoostConverter" H 6378 935 50  0000 L CNN
F 2 "" H 6350 1050 50  0001 C CNN
F 3 "~" H 6350 1050 50  0001 C CNN
	1    6350 1050
	1    0    0    -1  
$EndComp
Text GLabel 6000 950  0    50   Input ~ 0
4056OUT+
Wire Wire Line
	2300 1250 2450 1250
Wire Wire Line
	2300 1350 2400 1350
Wire Wire Line
	2300 1450 2400 1450
Wire Wire Line
	2400 1600 2400 1450
Connection ~ 2400 1450
Wire Wire Line
	2400 1450 2450 1450
Wire Wire Line
	2400 1450 2400 1350
Connection ~ 2400 1350
Wire Wire Line
	2400 1350 2450 1350
Text GLabel 6000 1050 0    50   Input ~ 0
4056OUT-
Wire Wire Line
	6000 950  6150 950 
Wire Wire Line
	6000 1050 6150 1050
Text GLabel 6000 1150 0    50   Input ~ 0
Boost5V
Wire Wire Line
	6000 1150 6150 1150
Text GLabel 6000 1250 0    50   Input ~ 0
BoostGND
Wire Wire Line
	6000 1250 6150 1250
$Comp
L Connector:Conn_01x08_Female J?
U 1 1 608A9E95
P 5750 3150
F 0 "J?" H 5778 3126 50  0000 L CNN
F 1 "HX711" H 5778 3035 50  0000 L CNN
F 2 "" H 5750 3150 50  0001 C CNN
F 3 "~" H 5750 3150 50  0001 C CNN
	1    5750 3150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 608AB91B
P 7150 3100
F 0 "J?" H 7178 3076 50  0000 L CNN
F 1 "HC-SR04" H 7178 2985 50  0000 L CNN
F 2 "" H 7150 3100 50  0001 C CNN
F 3 "~" H 7150 3100 50  0001 C CNN
	1    7150 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 608AC816
P 5700 4200
F 0 "J?" H 5728 4176 50  0000 L CNN
F 1 "MPU6050" H 5728 4085 50  0000 L CNN
F 2 "" H 5700 4200 50  0001 C CNN
F 3 "~" H 5700 4200 50  0001 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 608AD3B2
P 7150 4100
F 0 "J?" H 7178 4126 50  0000 L CNN
F 1 "FOTOELECTRIC_ENCODER" H 7178 4035 50  0000 L CNN
F 2 "" H 7150 4100 50  0001 C CNN
F 3 "~" H 7150 4100 50  0001 C CNN
	1    7150 4100
	1    0    0    -1  
$EndComp
Text GLabel 5200 2850 0    50   Input ~ 0
LC_E+
Text GLabel 5200 2950 0    50   Input ~ 0
LC_E-
Text GLabel 5200 3050 0    50   Input ~ 0
LC_A+
Text GLabel 5200 3150 0    50   Input ~ 0
LC_A-
Text GLabel 5200 3250 0    50   Input ~ 0
3.3Reg
$Comp
L power:Earth #PWR?
U 1 1 608AE540
P 5350 3700
F 0 "#PWR?" H 5350 3450 50  0001 C CNN
F 1 "Earth" H 5350 3550 50  0001 C CNN
F 2 "" H 5350 3700 50  0001 C CNN
F 3 "~" H 5350 3700 50  0001 C CNN
	1    5350 3700
	1    0    0    -1  
$EndComp
Text GLabel 5200 3450 0    50   Input ~ 0
LC_SCK
Text GLabel 5200 3550 0    50   Input ~ 0
LC_DT
Wire Wire Line
	5550 2850 5200 2850
Wire Wire Line
	5550 2950 5200 2950
Wire Wire Line
	5550 3050 5200 3050
Wire Wire Line
	5550 3150 5200 3150
Wire Wire Line
	5550 3250 5200 3250
Wire Wire Line
	5550 3350 5350 3350
Wire Wire Line
	5350 3350 5350 3700
Wire Wire Line
	5550 3450 5200 3450
Wire Wire Line
	5550 3550 5200 3550
Text GLabel 5250 4100 0    50   Input ~ 0
3.3Reg
$Comp
L power:Earth #PWR?
U 1 1 608B7DBB
P 5350 4600
F 0 "#PWR?" H 5350 4350 50  0001 C CNN
F 1 "Earth" H 5350 4450 50  0001 C CNN
F 2 "" H 5350 4600 50  0001 C CNN
F 3 "~" H 5350 4600 50  0001 C CNN
	1    5350 4600
	1    0    0    -1  
$EndComp
Text GLabel 5250 4200 0    50   Input ~ 0
MPU_SDA
Text GLabel 5250 4300 0    50   Input ~ 0
MPU_SCL
Wire Wire Line
	5500 4100 5250 4100
Wire Wire Line
	5500 4200 5250 4200
Wire Wire Line
	5500 4300 5250 4300
Wire Wire Line
	5500 4400 5350 4400
Wire Wire Line
	5350 4400 5350 4600
Text GLabel 6700 3100 0    50   Input ~ 0
US_TRIGGER
Text GLabel 6700 3200 0    50   Input ~ 0
US_ECHO
Text GLabel 6700 3000 0    50   Input ~ 0
3.3Reg
$Comp
L power:Earth #PWR?
U 1 1 608BE990
P 6700 3450
F 0 "#PWR?" H 6700 3200 50  0001 C CNN
F 1 "Earth" H 6700 3300 50  0001 C CNN
F 2 "" H 6700 3450 50  0001 C CNN
F 3 "~" H 6700 3450 50  0001 C CNN
	1    6700 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3000 6950 3000
Wire Wire Line
	6950 3100 6700 3100
Wire Wire Line
	6950 3200 6700 3200
Wire Wire Line
	6950 3300 6700 3300
Wire Wire Line
	6700 3300 6700 3450
Text GLabel 6650 4100 0    50   Input ~ 0
VEL_PULSE
Text GLabel 6650 4000 0    50   Input ~ 0
Boost5V
Text GLabel 6650 4200 0    50   Input ~ 0
BoostGND
Wire Wire Line
	6650 4000 6950 4000
Wire Wire Line
	6650 4100 6950 4100
Wire Wire Line
	6650 4200 6950 4200
$EndSCHEMATC
