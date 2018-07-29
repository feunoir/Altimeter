EESchema Schematic File Version 4
LIBS:Altimeter-cache
EELAYER 26 0
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
L Connector:Micro_SD_Card_Det_Hirose_DM3AT J3
U 1 1 5B543D39
P 10150 5650
F 0 "J3" H 10100 6467 50  0000 C CNN
F 1 "Micro_SD_Card_Det_Hirose_DM3AT" H 10100 6376 50  0000 C CNN
F 2 "Connectors_Hirose:microSD_Card_Receptacle_Hirose_DM3AT-SF-PEJM5" H 12200 6350 50  0001 C CNN
F 3 "https://www.hirose.com/product/en/download_file/key_name/DM3/category/Catalog/doc_file_id/49662/?file_category_id=4&item_id=195&is_series=1" H 10150 5750 50  0001 C CNN
	1    10150 5650
	1    0    0    -1  
$EndComp
Text Label 7650 5850 2    50   ~ 0
SD_DAT0
Text Label 7650 5450 2    50   ~ 0
SD_CMD
Text Label 7650 5650 2    50   ~ 0
SD_CLK
NoConn ~ 9250 6050
NoConn ~ 9250 6150
$Comp
L power:GND #PWR0101
U 1 1 5B544ED9
P 11050 6250
F 0 "#PWR0101" H 11050 6000 50  0001 C CNN
F 1 "GND" H 11055 6077 50  0000 C CNN
F 2 "" H 11050 6250 50  0001 C CNN
F 3 "" H 11050 6250 50  0001 C CNN
	1    11050 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 6250 11050 6150
Wire Wire Line
	11050 6150 10950 6150
$Comp
L power:+3.3V #PWR0102
U 1 1 5B5450AD
P 9050 4750
F 0 "#PWR0102" H 9050 4600 50  0001 C CNN
F 1 "+3.3V" H 9065 4923 50  0000 C CNN
F 2 "" H 9050 4750 50  0001 C CNN
F 3 "" H 9050 4750 50  0001 C CNN
	1    9050 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5B54517A
P 9150 6250
F 0 "#PWR0103" H 9150 6000 50  0001 C CNN
F 1 "GND" H 9155 6077 50  0000 C CNN
F 2 "" H 9150 6250 50  0001 C CNN
F 3 "" H 9150 6250 50  0001 C CNN
	1    9150 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 6250 9150 5750
Text Label 5250 4650 2    50   ~ 0
SD_DAT0
Text Label 5250 5050 2    50   ~ 0
SD_CLK
Text Label 5250 3650 2    50   ~ 0
SD_CMD
Text Label 6550 3350 0    50   ~ 0
SWDIO
Text Label 6550 3450 0    50   ~ 0
SWCLK
Text Label 6550 2350 0    50   ~ 0
DEBUG_TX
Text Label 6550 2450 0    50   ~ 0
DEBUG_RX
Text Label 6550 4850 0    50   ~ 0
RS485_TX
Text Label 6550 4950 0    50   ~ 0
RS485_RX
Text Label 6550 4650 0    50   ~ 0
I2C1_SCL
Text Label 6550 4750 0    50   ~ 0
I2C1_SDA
Text Label 5250 3250 2    50   ~ 0
HSE_IN
Text Label 5250 3350 2    50   ~ 0
HSE_OUT
Text Label 5250 5250 2    50   ~ 0
LSE_IN
Text Label 5250 5350 2    50   ~ 0
LSE_OUT
Text Label 5250 2150 2    50   ~ 0
RESET
$Comp
L power:GND #PWR0104
U 1 1 5B54592A
P 5850 5850
F 0 "#PWR0104" H 5850 5600 50  0001 C CNN
F 1 "GND" H 5855 5677 50  0000 C CNN
F 2 "" H 5850 5850 50  0001 C CNN
F 3 "" H 5850 5850 50  0001 C CNN
	1    5850 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5550 5650 5750
Wire Wire Line
	5650 5750 5750 5750
Wire Wire Line
	5850 5750 5850 5850
Wire Wire Line
	5850 5750 5850 5550
Connection ~ 5850 5750
Wire Wire Line
	5750 5550 5750 5750
Connection ~ 5750 5750
Wire Wire Line
	5750 5750 5850 5750
Wire Wire Line
	5850 5750 5950 5750
Wire Wire Line
	5950 5750 5950 5550
Wire Wire Line
	5950 5750 6050 5750
Connection ~ 5950 5750
Wire Wire Line
	6050 5550 6050 5750
$Comp
L Device:C_Small C8
U 1 1 5B54665A
P 5250 1650
F 0 "C8" H 5342 1696 50  0000 L CNN
F 1 "0.1u" H 5342 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5250 1650 50  0001 C CNN
F 3 "~" H 5250 1650 50  0001 C CNN
	1    5250 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5B54673C
P 5900 1100
F 0 "#PWR0105" H 5900 950 50  0001 C CNN
F 1 "+3.3V" H 5915 1273 50  0000 C CNN
F 2 "" H 5900 1100 50  0001 C CNN
F 3 "" H 5900 1100 50  0001 C CNN
	1    5900 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5B546B09
P 5250 1750
F 0 "#PWR0106" H 5250 1500 50  0001 C CNN
F 1 "GND" H 5255 1577 50  0000 C CNN
F 2 "" H 5250 1750 50  0001 C CNN
F 3 "" H 5250 1750 50  0001 C CNN
	1    5250 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5B5485AA
P 4950 1650
F 0 "C7" H 5042 1696 50  0000 L CNN
F 1 "0.1u" H 5042 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4950 1650 50  0001 C CNN
F 3 "~" H 4950 1650 50  0001 C CNN
	1    4950 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5B549555
P 4650 1650
F 0 "C6" H 4742 1696 50  0000 L CNN
F 1 "0.1u" H 4742 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4650 1650 50  0001 C CNN
F 3 "~" H 4650 1650 50  0001 C CNN
	1    4650 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5B54955B
P 4650 1750
F 0 "#PWR0107" H 4650 1500 50  0001 C CNN
F 1 "GND" H 4655 1577 50  0000 C CNN
F 2 "" H 4650 1750 50  0001 C CNN
F 3 "" H 4650 1750 50  0001 C CNN
	1    4650 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1550 4650 1450
$Comp
L Device:C_Small C5
U 1 1 5B54C69C
P 4350 1650
F 0 "C5" H 4442 1696 50  0000 L CNN
F 1 "0.1u" H 4442 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4350 1650 50  0001 C CNN
F 3 "~" H 4350 1650 50  0001 C CNN
	1    4350 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5B54C6A2
P 4350 1750
F 0 "#PWR0108" H 4350 1500 50  0001 C CNN
F 1 "GND" H 4355 1577 50  0000 C CNN
F 2 "" H 4350 1750 50  0001 C CNN
F 3 "" H 4350 1750 50  0001 C CNN
	1    4350 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1550 4350 1400
Wire Wire Line
	5650 1300 5750 1300
Connection ~ 5850 1300
Wire Wire Line
	5850 1300 5900 1300
Connection ~ 5750 1300
Wire Wire Line
	5750 1300 5850 1300
Wire Wire Line
	5900 1300 5950 1300
Connection ~ 5900 1300
Wire Wire Line
	5950 1300 6150 1300
Connection ~ 5950 1300
$Comp
L Device:C_Small C4
U 1 1 5B55198B
P 4050 1650
F 0 "C4" H 4142 1696 50  0000 L CNN
F 1 "4.7u" H 4142 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4050 1650 50  0001 C CNN
F 3 "~" H 4050 1650 50  0001 C CNN
	1    4050 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5B551991
P 4050 1750
F 0 "#PWR0109" H 4050 1500 50  0001 C CNN
F 1 "GND" H 4055 1577 50  0000 C CNN
F 2 "" H 4050 1750 50  0001 C CNN
F 3 "" H 4050 1750 50  0001 C CNN
	1    4050 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5B553D6E
P 6550 1650
F 0 "C9" H 6642 1696 50  0000 L CNN
F 1 "1u" H 6642 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6550 1650 50  0001 C CNN
F 3 "~" H 6550 1650 50  0001 C CNN
	1    6550 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5B553DE6
P 6850 1650
F 0 "C10" H 6942 1696 50  0000 L CNN
F 1 "0.01u" H 6942 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6850 1650 50  0001 C CNN
F 3 "~" H 6850 1650 50  0001 C CNN
	1    6850 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5B556593
P 6550 1750
F 0 "#PWR0110" H 6550 1500 50  0001 C CNN
F 1 "GND" H 6555 1577 50  0000 C CNN
F 2 "" H 6550 1750 50  0001 C CNN
F 3 "" H 6550 1750 50  0001 C CNN
	1    6550 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5B5565B7
P 6850 1750
F 0 "#PWR0111" H 6850 1500 50  0001 C CNN
F 1 "GND" H 6855 1577 50  0000 C CNN
F 2 "" H 6850 1750 50  0001 C CNN
F 3 "" H 6850 1750 50  0001 C CNN
	1    6850 1750
	1    0    0    -1  
$EndComp
Text Label 6550 4450 0    50   ~ 0
GNSS_TX
Text Label 6550 4550 0    50   ~ 0
GNSS_RX
$Comp
L Device:LED_Small D3
U 1 1 5B5578DA
P 10150 2950
F 0 "D3" H 10100 2900 50  0000 R CNN
F 1 "LED_Y" H 10200 2900 50  0000 L CNN
F 2 "LEDs:LED_0603" V 10150 2950 50  0001 C CNN
F 3 "~" V 10150 2950 50  0001 C CNN
	1    10150 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D4
U 1 1 5B558A2F
P 10500 3050
F 0 "D4" H 10450 3000 50  0000 R CNN
F 1 "LED_G" H 10550 3000 50  0000 L CNN
F 2 "LEDs:LED_0603" V 10500 3050 50  0001 C CNN
F 3 "~" V 10500 3050 50  0001 C CNN
	1    10500 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 5B55982D
P 10150 3150
F 0 "D5" H 10100 3100 50  0000 R CNN
F 1 "LED_B" H 10200 3100 50  0000 L CNN
F 2 "LEDs:LED_0603" V 10150 3150 50  0001 C CNN
F 3 "~" V 10150 3150 50  0001 C CNN
	1    10150 3150
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5B55B83E
P 10500 2950
F 0 "R3" V 10450 3050 50  0000 L CNN
F 1 "300" V 10500 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10430 2950 50  0001 C CNN
F 3 "~" H 10500 2950 50  0001 C CNN
	1    10500 2950
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5B55B94B
P 10150 3050
F 0 "R4" V 10100 2950 50  0000 R CNN
F 1 "300" V 10150 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10080 3050 50  0001 C CNN
F 3 "~" H 10150 3050 50  0001 C CNN
	1    10150 3050
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5B55B977
P 10500 3150
F 0 "R5" V 10450 3250 50  0000 L CNN
F 1 "300" V 10500 3150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10430 3150 50  0001 C CNN
F 3 "~" H 10500 3150 50  0001 C CNN
	1    10500 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	10350 3150 10250 3150
Wire Wire Line
	10350 2950 10250 2950
$Comp
L power:GND #PWR0112
U 1 1 5B55E57E
P 10750 3250
F 0 "#PWR0112" H 10750 3000 50  0001 C CNN
F 1 "GND" H 10755 3077 50  0000 C CNN
F 2 "" H 10750 3250 50  0001 C CNN
F 3 "" H 10750 3250 50  0001 C CNN
	1    10750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 3250 10750 3150
Wire Wire Line
	10750 3150 10650 3150
Wire Wire Line
	10750 3150 10750 3050
Connection ~ 10750 3150
Wire Wire Line
	10750 3050 10750 2950
Wire Wire Line
	10750 2950 10650 2950
Connection ~ 10750 3050
Wire Wire Line
	10050 2950 9750 2950
NoConn ~ 6550 4250
NoConn ~ 6550 4150
NoConn ~ 6550 3950
NoConn ~ 6550 3550
NoConn ~ 6550 3650
NoConn ~ 6550 2950
NoConn ~ 6550 2750
NoConn ~ 6550 2650
NoConn ~ 6550 2550
NoConn ~ 6550 2250
NoConn ~ 6550 2150
NoConn ~ 5250 3450
NoConn ~ 5250 3850
NoConn ~ 5250 3950
NoConn ~ 5250 4050
NoConn ~ 5250 4150
NoConn ~ 5250 4450
NoConn ~ 5250 4550
$Comp
L power:+3.3V #PWR0113
U 1 1 5B590996
P 6250 1100
F 0 "#PWR0113" H 6250 950 50  0001 C CNN
F 1 "+3.3V" H 6265 1273 50  0000 C CNN
F 2 "" H 6250 1100 50  0001 C CNN
F 3 "" H 6250 1100 50  0001 C CNN
	1    6250 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1200 4050 1200
Connection ~ 5900 1200
Wire Wire Line
	5900 1200 5900 1300
Wire Wire Line
	5900 1100 5900 1200
Wire Wire Line
	6150 1300 6150 1550
Wire Wire Line
	5250 1550 6150 1550
Connection ~ 6150 1550
$Comp
L power:GND #PWR0114
U 1 1 5B5485B0
P 4950 1750
F 0 "#PWR0114" H 4950 1500 50  0001 C CNN
F 1 "GND" H 4955 1577 50  0000 C CNN
F 2 "" H 4950 1750 50  0001 C CNN
F 3 "" H 4950 1750 50  0001 C CNN
	1    4950 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1550 4950 1500
Wire Wire Line
	5750 1300 5750 1400
Connection ~ 5750 1400
Wire Wire Line
	4350 1400 5750 1400
Wire Wire Line
	5850 1300 5850 1450
Connection ~ 5850 1450
Wire Wire Line
	5850 1450 4650 1450
Wire Wire Line
	5950 1300 5950 1500
Connection ~ 5950 1500
Wire Wire Line
	4950 1500 5950 1500
Wire Wire Line
	6550 1450 6550 1550
Wire Wire Line
	6550 1450 6850 1450
Wire Wire Line
	6850 1450 6850 1550
Connection ~ 6550 1450
Wire Wire Line
	6050 1450 6250 1450
Wire Wire Line
	6250 1450 6250 1100
Connection ~ 6250 1450
Wire Wire Line
	6250 1450 6550 1450
$Comp
L KUT_CommIC:ST1480AxDR U3
U 1 1 5B5A4983
P 9200 1850
F 0 "U3" H 9500 2300 50  0000 C CNN
F 1 "ST1480AxDR" H 9500 2200 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 10250 1500 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/81/93/fa/ed/87/47/46/00/CD00003136.pdf/files/CD00003136.pdf/jcr:content/translations/en.CD00003136.pdf" H 8900 2200 50  0001 C CNN
	1    9200 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0115
U 1 1 5B5A4A57
P 9200 1450
F 0 "#PWR0115" H 9200 1300 50  0001 C CNN
F 1 "+3.3V" H 9215 1623 50  0000 C CNN
F 2 "" H 9200 1450 50  0001 C CNN
F 3 "" H 9200 1450 50  0001 C CNN
	1    9200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0116
U 1 1 5B5A4BA6
P 8700 1250
F 0 "#PWR0116" H 8700 1100 50  0001 C CNN
F 1 "+3.3V" H 8715 1423 50  0000 C CNN
F 2 "" H 8700 1250 50  0001 C CNN
F 3 "" H 8700 1250 50  0001 C CNN
	1    8700 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1750 8800 1750
Wire Wire Line
	8700 1750 8700 1850
Wire Wire Line
	8700 1950 8800 1950
Text Label 8600 2050 2    50   ~ 0
RS485_TX
$Comp
L Device:R_Small R13
U 1 1 5B5A8C9E
P 9800 1850
F 0 "R13" H 9859 1896 50  0000 L CNN
F 1 "120" H 9859 1805 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9800 1850 50  0001 C CNN
F 3 "~" H 9800 1850 50  0001 C CNN
	1    9800 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 1750 9600 1750
Wire Wire Line
	9600 1950 9700 1950
$Comp
L power:GND #PWR0117
U 1 1 5B5ABA7C
P 9200 2250
F 0 "#PWR0117" H 9200 2000 50  0001 C CNN
F 1 "GND" H 9205 2077 50  0000 C CNN
F 2 "" H 9200 2250 50  0001 C CNN
F 3 "" H 9200 2250 50  0001 C CNN
	1    9200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 1700 9700 1700
Wire Wire Line
	9700 1700 9700 1750
Wire Wire Line
	9800 1750 9800 1700
Wire Wire Line
	9800 1950 9800 2000
Wire Wire Line
	9800 2000 9700 2000
Wire Wire Line
	9700 2000 9700 1950
$Comp
L power:GND #PWR0118
U 1 1 5B5B7B17
P 10300 2000
F 0 "#PWR0118" H 10300 1750 50  0001 C CNN
F 1 "GND" H 10305 1827 50  0000 C CNN
F 2 "" H 10300 2000 50  0001 C CNN
F 3 "" H 10300 2000 50  0001 C CNN
	1    10300 2000
	1    0    0    -1  
$EndComp
Connection ~ 9800 2000
Connection ~ 9800 1700
$Comp
L power:+3.3V #PWR0119
U 1 1 5B5BF552
P 10300 1500
F 0 "#PWR0119" H 10300 1350 50  0001 C CNN
F 1 "+3.3V" H 10315 1673 50  0000 C CNN
F 2 "" H 10300 1500 50  0001 C CNN
F 3 "" H 10300 1500 50  0001 C CNN
	1    10300 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1800 10100 1700
Wire Wire Line
	10100 1800 10400 1800
Wire Wire Line
	10100 1900 10100 2000
Wire Wire Line
	10100 1900 10400 1900
Wire Wire Line
	9800 2000 10100 2000
Wire Wire Line
	9800 1700 10100 1700
Text Label 1100 3100 0    50   ~ 0
GNSS_TX
Text Label 1100 3000 0    50   ~ 0
GNSS_RX
$Comp
L power:GND #PWR0120
U 1 1 5B5CBB21
P 1000 3300
F 0 "#PWR0120" H 1000 3050 50  0001 C CNN
F 1 "GND" H 1005 3127 50  0000 C CNN
F 2 "" H 1000 3300 50  0001 C CNN
F 3 "" H 1000 3300 50  0001 C CNN
	1    1000 3300
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0121
U 1 1 5B5CFCF6
P 1000 2700
F 0 "#PWR0121" H 1000 2550 50  0001 C CNN
F 1 "+3.3V" H 1015 2873 50  0000 C CNN
F 2 "" H 1000 2700 50  0001 C CNN
F 3 "" H 1000 2700 50  0001 C CNN
	1    1000 2700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	900  3000 1100 3000
Wire Wire Line
	1100 3100 900  3100
Text Label 1100 3200 0    50   ~ 0
GNSS_1PPS
Wire Wire Line
	1100 3200 900  3200
Text Label 6550 4350 0    50   ~ 0
GNSS_1PPS
$Comp
L Connector:Conn_01x06_Male J2
U 1 1 5B5DD216
P 700 4100
F 0 "J2" H 806 4478 50  0000 C CNN
F 1 "STLink" H 806 4387 50  0000 C CNN
F 2 "temp:DF11-6DP-2DSA" H 700 4100 50  0001 C CNN
F 3 "~" H 700 4100 50  0001 C CNN
	1    700  4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5B5DD3D5
P 1000 4500
F 0 "#PWR0122" H 1000 4250 50  0001 C CNN
F 1 "GND" H 1005 4327 50  0000 C CNN
F 2 "" H 1000 4500 50  0001 C CNN
F 3 "" H 1000 4500 50  0001 C CNN
	1    1000 4500
	1    0    0    -1  
$EndComp
Text Label 1100 4100 0    50   ~ 0
SWDIO
Text Label 1100 4200 0    50   ~ 0
SWCLK
Text Label 1100 4400 0    50   ~ 0
DEBUG_TX
Text Label 1100 4300 0    50   ~ 0
DEBUG_RX
$Comp
L KUT_Sensor:LPS22HB_Module U2
U 1 1 5B5E98AF
P 2350 6900
F 0 "U2" H 2350 7337 60  0000 C CNN
F 1 "LPS22HB_Module" H 2350 7231 60  0000 C CNN
F 2 "KUT_Sensor:Strawberry-Linux_Module_8P_NoHole" H 2250 7100 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/bf/c1/4f/23/61/17/44/8a/DM00140895.pdf/files/DM00140895.pdf/jcr:content/translations/en.DM00140895.pdf" H 2350 7200 60  0001 C CNN
F 4 "https://strawberry-linux.com/catalog/items?code=12122" H 2450 7300 60  0001 C CNN "製品ページ"
	1    2350 6900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0123
U 1 1 5B5EC11A
P 1650 6650
F 0 "#PWR0123" H 1650 6500 50  0001 C CNN
F 1 "+3.3V" H 1665 6823 50  0000 C CNN
F 2 "" H 1650 6650 50  0001 C CNN
F 3 "" H 1650 6650 50  0001 C CNN
	1    1650 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6650 1650 6750
Wire Wire Line
	1650 6750 1750 6750
$Comp
L power:+3.3V #PWR0124
U 1 1 5B5EE7C2
P 3050 6650
F 0 "#PWR0124" H 3050 6500 50  0001 C CNN
F 1 "+3.3V" H 3065 6823 50  0000 C CNN
F 2 "" H 3050 6650 50  0001 C CNN
F 3 "" H 3050 6650 50  0001 C CNN
	1    3050 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6650 3050 6750
Wire Wire Line
	3050 6750 2950 6750
Wire Wire Line
	2950 6850 3050 6850
Wire Wire Line
	3050 6850 3050 6750
Connection ~ 3050 6750
NoConn ~ 2950 6950
$Comp
L power:GND #PWR0125
U 1 1 5B5F622E
P 3050 7150
F 0 "#PWR0125" H 3050 6900 50  0001 C CNN
F 1 "GND" H 3055 6977 50  0000 C CNN
F 2 "" H 3050 7150 50  0001 C CNN
F 3 "" H 3050 7150 50  0001 C CNN
	1    3050 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 7150 3050 7050
Wire Wire Line
	3050 7050 2950 7050
$Comp
L power:GND #PWR0126
U 1 1 5B5F8D0F
P 1650 7150
F 0 "#PWR0126" H 1650 6900 50  0001 C CNN
F 1 "GND" H 1655 6977 50  0000 C CNN
F 2 "" H 1650 7150 50  0001 C CNN
F 3 "" H 1650 7150 50  0001 C CNN
	1    1650 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 7150 1650 7050
Wire Wire Line
	1650 7050 1750 7050
Text Label 1150 6850 2    50   ~ 0
I2C1_SCL
Text Label 1150 6950 2    50   ~ 0
I2C1_SDA
$Comp
L KUT_PowerIC:NJM12888 VR1
U 1 1 5B5FEB62
P 2050 1400
F 0 "VR1" H 2050 1787 60  0000 C CNN
F 1 "NJM12888" H 2050 1681 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 2050 1000 60  0001 C CNN
F 3 "http://www.njr.co.jp/products/semicon/PDF/NJM12888_J.pdf" H 2050 900 60  0001 C CNN
	1    2050 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5B5FEDDE
P 1550 1700
F 0 "C1" H 1642 1746 50  0000 L CNN
F 1 "0.1u" H 1642 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1550 1700 50  0001 C CNN
F 3 "~" H 1550 1700 50  0001 C CNN
	1    1550 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5B5FEE6A
P 2550 1700
F 0 "C2" H 2642 1746 50  0000 L CNN
F 1 "0.47u" H 2642 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2550 1700 50  0001 C CNN
F 3 "~" H 2550 1700 50  0001 C CNN
	1    2550 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5B5FEF27
P 2550 1800
F 0 "#PWR0127" H 2550 1550 50  0001 C CNN
F 1 "GND" H 2555 1627 50  0000 C CNN
F 2 "" H 2550 1800 50  0001 C CNN
F 3 "" H 2550 1800 50  0001 C CNN
	1    2550 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5B5FEF69
P 1550 1800
F 0 "#PWR0128" H 1550 1550 50  0001 C CNN
F 1 "GND" H 1555 1627 50  0000 C CNN
F 2 "" H 1550 1800 50  0001 C CNN
F 3 "" H 1550 1800 50  0001 C CNN
	1    1550 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5B5FEFA4
P 2050 1800
F 0 "#PWR0129" H 2050 1550 50  0001 C CNN
F 1 "GND" H 2055 1627 50  0000 C CNN
F 2 "" H 2050 1800 50  0001 C CNN
F 3 "" H 2050 1800 50  0001 C CNN
	1    2050 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1800 2050 1700
NoConn ~ 2450 1500
Wire Wire Line
	2450 1300 2550 1300
Wire Wire Line
	2550 1300 2550 1600
$Comp
L power:+3.3V #PWR0130
U 1 1 5B60FEBC
P 2550 1200
F 0 "#PWR0130" H 2550 1050 50  0001 C CNN
F 1 "+3.3V" H 2565 1373 50  0000 C CNN
F 2 "" H 2550 1200 50  0001 C CNN
F 3 "" H 2550 1200 50  0001 C CNN
	1    2550 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1200 2550 1300
Connection ~ 2550 1300
$Comp
L power:+6V #PWR0131
U 1 1 5B612F94
P 1550 1200
F 0 "#PWR0131" H 1550 1050 50  0001 C CNN
F 1 "+6V" H 1565 1373 50  0000 C CNN
F 2 "" H 1550 1200 50  0001 C CNN
F 3 "" H 1550 1200 50  0001 C CNN
	1    1550 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1200 1550 1300
Wire Wire Line
	1550 1300 1650 1300
Wire Wire Line
	1550 1300 1550 1500
Connection ~ 1550 1300
Wire Wire Line
	1550 1500 1650 1500
Connection ~ 1550 1500
Wire Wire Line
	1550 1500 1550 1600
$Comp
L power:+3.3V #PWR0132
U 1 1 5B61CD9C
P 2950 1200
F 0 "#PWR0132" H 2950 1050 50  0001 C CNN
F 1 "+3.3V" H 2965 1373 50  0000 C CNN
F 2 "" H 2950 1200 50  0001 C CNN
F 3 "" H 2950 1200 50  0001 C CNN
	1    2950 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 5B61CE0A
P 2950 1300
F 0 "D1" V 2996 1232 50  0000 R CNN
F 1 "PWR" V 2905 1232 50  0000 R CNN
F 2 "LEDs:LED_0603" V 2950 1300 50  0001 C CNN
F 3 "~" V 2950 1300 50  0001 C CNN
	1    2950 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5B61CF71
P 2950 1500
F 0 "R1" H 3009 1546 50  0000 L CNN
F 1 "300" H 3009 1455 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 2950 1500 50  0001 C CNN
F 3 "~" H 2950 1500 50  0001 C CNN
	1    2950 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5B61D024
P 2950 1600
F 0 "#PWR0133" H 2950 1350 50  0001 C CNN
F 1 "GND" H 2955 1427 50  0000 C CNN
F 2 "" H 2950 1600 50  0001 C CNN
F 3 "" H 2950 1600 50  0001 C CNN
	1    2950 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR0134
U 1 1 5B6208C2
P 1000 1200
F 0 "#PWR0134" H 1000 1050 50  0001 C CNN
F 1 "+6V" H 1015 1373 50  0000 C CNN
F 2 "" H 1000 1200 50  0001 C CNN
F 3 "" H 1000 1200 50  0001 C CNN
	1    1000 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 1200 1000 1300
Wire Wire Line
	1000 1300 900  1300
$Comp
L power:GND #PWR0135
U 1 1 5B6242B1
P 1000 1500
F 0 "#PWR0135" H 1000 1250 50  0001 C CNN
F 1 "GND" H 1005 1327 50  0000 C CNN
F 2 "" H 1000 1500 50  0001 C CNN
F 3 "" H 1000 1500 50  0001 C CNN
	1    1000 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 1500 1000 1400
Wire Wire Line
	1000 1400 900  1400
$Comp
L Device:C_Small C3
U 1 1 5B62FF8A
P 4450 2700
F 0 "C3" H 4542 2746 50  0000 L CNN
F 1 "0.1u" H 4542 2655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4450 2700 50  0001 C CNN
F 3 "~" H 4450 2700 50  0001 C CNN
	1    4450 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2600 4450 2500
$Comp
L power:GND #PWR0136
U 1 1 5B64363A
P 4450 2800
F 0 "#PWR0136" H 4450 2550 50  0001 C CNN
F 1 "GND" H 4455 2627 50  0000 C CNN
F 2 "" H 4450 2800 50  0001 C CNN
F 3 "" H 4450 2800 50  0001 C CNN
	1    4450 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 5B643696
P 3850 3200
F 0 "#PWR0137" H 3850 2950 50  0001 C CNN
F 1 "GND" H 3855 3027 50  0000 C CNN
F 2 "" H 3850 3200 50  0001 C CNN
F 3 "" H 3850 3200 50  0001 C CNN
	1    3850 3200
	1    0    0    -1  
$EndComp
Text Label 4550 2500 0    50   ~ 0
RESET
Wire Wire Line
	4550 2500 4450 2500
Wire Wire Line
	10050 3150 9750 3150
Text Label 9750 2950 2    50   ~ 0
LED_Y
Text Label 9750 3050 2    50   ~ 0
LED_G
Text Label 9750 3150 2    50   ~ 0
LED_B
Text Label 6550 5050 0    50   ~ 0
LED_R
Text Label 6550 5250 0    50   ~ 0
LED_G
Text Label 6550 5350 0    50   ~ 0
LED_B
Text Label 7650 5950 2    50   ~ 0
SD_DAT1
Text Label 7650 5250 2    50   ~ 0
SD_DAT2
Text Label 7650 5350 2    50   ~ 0
SD_DAT3
$Comp
L Device:R_Small R10
U 1 1 5B6B82C9
P 8600 5050
F 0 "R10" H 8659 5096 50  0000 L CNN
F 1 "22k" H 8659 5005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8600 5050 50  0001 C CNN
F 3 "~" H 8600 5050 50  0001 C CNN
	1    8600 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5B6B8405
P 8850 5050
F 0 "R11" H 8909 5096 50  0000 L CNN
F 1 "22k" H 8909 5005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8850 5050 50  0001 C CNN
F 3 "~" H 8850 5050 50  0001 C CNN
	1    8850 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5B6C1E15
P 8100 5050
F 0 "R8" H 8159 5096 50  0000 L CNN
F 1 "22k" H 8159 5005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8100 5050 50  0001 C CNN
F 3 "~" H 8100 5050 50  0001 C CNN
	1    8100 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5B6C1E1B
P 8350 5050
F 0 "R9" H 8409 5096 50  0000 L CNN
F 1 "22k" H 8409 5005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8350 5050 50  0001 C CNN
F 3 "~" H 8350 5050 50  0001 C CNN
	1    8350 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 4750 9050 4850
Wire Wire Line
	9050 4850 8850 4850
Wire Wire Line
	8850 4850 8850 4950
Connection ~ 9050 4850
Wire Wire Line
	9050 4850 9050 5550
Wire Wire Line
	8600 5850 8600 5150
Wire Wire Line
	8350 5450 8350 5150
Wire Wire Line
	8850 5150 8850 5950
Wire Wire Line
	8100 5350 8100 5150
$Comp
L Device:R_Small R7
U 1 1 5B6EE9DB
P 7850 5050
F 0 "R7" H 7909 5096 50  0000 L CNN
F 1 "22k" H 7909 5005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 7850 5050 50  0001 C CNN
F 3 "~" H 7850 5050 50  0001 C CNN
	1    7850 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5150 7850 5250
Wire Wire Line
	8850 4850 8600 4850
Wire Wire Line
	8600 4850 8600 4950
Connection ~ 8850 4850
Wire Wire Line
	8600 4850 8350 4850
Wire Wire Line
	8350 4850 8350 4950
Connection ~ 8600 4850
Wire Wire Line
	8350 4850 8100 4850
Wire Wire Line
	8100 4850 8100 4950
Connection ~ 8350 4850
Wire Wire Line
	8100 4850 7850 4850
Wire Wire Line
	7850 4850 7850 4950
Connection ~ 8100 4850
Wire Wire Line
	8600 5850 9250 5850
Wire Wire Line
	8850 5950 9250 5950
Wire Wire Line
	9050 5550 9250 5550
Wire Wire Line
	8350 5450 9250 5450
Wire Wire Line
	8100 5350 9250 5350
Wire Wire Line
	7850 5250 9250 5250
Wire Wire Line
	9250 5750 9150 5750
$Comp
L Device:C_Small C15
U 1 1 5B757202
P 9150 5050
F 0 "C15" H 9242 5096 50  0000 L CNN
F 1 "1u" H 9242 5005 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 9150 5050 50  0001 C CNN
F 3 "~" H 9150 5050 50  0001 C CNN
	1    9150 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 4850 9150 4850
Wire Wire Line
	9150 4850 9150 4950
Wire Wire Line
	9150 5150 9150 5750
Connection ~ 9150 5750
Connection ~ 7850 5250
Connection ~ 8100 5350
Connection ~ 8350 5450
Connection ~ 8600 5850
Connection ~ 8850 5950
Wire Wire Line
	7650 5450 8350 5450
Wire Wire Line
	7650 5350 8100 5350
Wire Wire Line
	7650 5250 7850 5250
Wire Wire Line
	7650 5850 8600 5850
Wire Wire Line
	7650 5950 8850 5950
Wire Wire Line
	5650 1300 5650 1950
Wire Wire Line
	5750 1400 5750 1950
Wire Wire Line
	5850 1450 5850 1950
Wire Wire Line
	5950 1500 5950 1950
Wire Wire Line
	6050 1450 6050 1950
Wire Wire Line
	6150 1550 6150 1950
Wire Wire Line
	4050 1200 4050 1550
Text Notes 7950 5650 0    30   ~ 0
As close as possible\nto signal source
$Comp
L Device:R R12
U 1 1 5B820D72
P 7800 5650
F 0 "R12" V 7850 5750 50  0000 L CNN
F 1 "0-47" V 7800 5650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7730 5650 50  0001 C CNN
F 3 "~" H 7800 5650 50  0001 C CNN
	1    7800 5650
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 5650 9250 5650
Text Label 6550 5150 0    50   ~ 0
LED_Y
$Comp
L Device:LED_Small D2
U 1 1 5B82EC8C
P 10500 2850
F 0 "D2" H 10450 2800 50  0000 R CNN
F 1 "LED_R" H 10550 2800 50  0000 L CNN
F 2 "LEDs:LED_0603" V 10500 2850 50  0001 C CNN
F 3 "~" V 10500 2850 50  0001 C CNN
	1    10500 2850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5B82EC92
P 10150 2850
F 0 "R2" V 10100 2750 50  0000 R CNN
F 1 "300" V 10150 2850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10080 2850 50  0001 C CNN
F 3 "~" H 10150 2850 50  0001 C CNN
	1    10150 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	10750 2950 10750 2850
Text Label 9750 2850 2    50   ~ 0
LED_R
Connection ~ 10750 2950
Text Label 5250 4850 2    50   ~ 0
SD_DAT2
Text Label 5250 4950 2    50   ~ 0
SD_DAT3
Text Label 5250 4750 2    50   ~ 0
SD_DAT1
NoConn ~ 5250 5150
Wire Wire Line
	1100 4000 900  4000
Wire Wire Line
	900  4100 1100 4100
Wire Wire Line
	1100 4300 900  4300
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5B87633D
P 4350 6800
F 0 "Y1" H 4200 6950 50  0000 R CNN
F 1 "FA238-20MHz" H 4200 6900 40  0000 R CNN
F 2 "Crystals:Crystal_SMD_SeikoEpson_FA238-4pin_3.2x2.5mm" H 4350 6800 50  0001 C CNN
F 3 "~" H 4350 6800 50  0001 C CNN
	1    4350 6800
	1    0    0    -1  
$EndComp
Text Label 4000 6800 2    50   ~ 0
HSE_IN
Text Label 5000 6800 0    50   ~ 0
HSE_OUT
$Comp
L Device:C_Small C11
U 1 1 5B8767D4
P 4100 7000
F 0 "C11" H 4000 7050 50  0000 R CNN
F 1 "12p" H 4000 6950 50  0000 R CNN
F 2 "Capacitors_SMD:C_0603" H 4100 7000 50  0001 C CNN
F 3 "~" H 4100 7000 50  0001 C CNN
	1    4100 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 6800 4100 6800
Wire Wire Line
	4100 6800 4100 6900
Connection ~ 4100 6800
Wire Wire Line
	4100 6800 4200 6800
$Comp
L Device:C_Small C12
U 1 1 5B8858DB
P 4600 7000
F 0 "C12" H 4700 7050 50  0000 L CNN
F 1 "12p" H 4700 6950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4600 7000 50  0001 C CNN
F 3 "~" H 4600 7000 50  0001 C CNN
	1    4600 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 6800 4600 6800
Wire Wire Line
	4600 6800 4600 6900
Wire Wire Line
	4600 6800 4500 6800
Connection ~ 4600 6800
$Comp
L power:GND #PWR0138
U 1 1 5B8957B4
P 4100 7100
F 0 "#PWR0138" H 4100 6850 50  0001 C CNN
F 1 "GND" H 4105 6927 50  0000 C CNN
F 2 "" H 4100 7100 50  0001 C CNN
F 3 "" H 4100 7100 50  0001 C CNN
	1    4100 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 5B895818
P 4350 7100
F 0 "#PWR0139" H 4350 6850 50  0001 C CNN
F 1 "GND" H 4355 6927 50  0000 C CNN
F 2 "" H 4350 7100 50  0001 C CNN
F 3 "" H 4350 7100 50  0001 C CNN
	1    4350 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 5B895875
P 4600 7100
F 0 "#PWR0140" H 4600 6850 50  0001 C CNN
F 1 "GND" H 4605 6927 50  0000 C CNN
F 2 "" H 4600 7100 50  0001 C CNN
F 3 "" H 4600 7100 50  0001 C CNN
	1    4600 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 7100 4350 7000
$Comp
L power:GND #PWR0141
U 1 1 5B89DA8D
P 4600 6550
F 0 "#PWR0141" H 4600 6300 50  0001 C CNN
F 1 "GND" H 4605 6377 50  0000 C CNN
F 2 "" H 4600 6550 50  0001 C CNN
F 3 "" H 4600 6550 50  0001 C CNN
	1    4600 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6550 4600 6500
Wire Wire Line
	4600 6500 4350 6500
Wire Wire Line
	4350 6500 4350 6600
$Comp
L Device:R R6
U 1 1 5B8AF831
P 4850 6800
F 0 "R6" V 4750 6800 50  0000 C CNN
F 1 "80" V 4850 6800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4780 6800 50  0001 C CNN
F 3 "~" H 4850 6800 50  0001 C CNN
	1    4850 6800
	0    1    1    0   
$EndComp
$Comp
L Device:Crystal Y2
U 1 1 5B8AFFB3
P 6000 6800
F 0 "Y2" H 6000 7068 50  0000 C CNN
F 1 "FC255-32.768kHz" H 6000 6977 50  0000 C CNN
F 2 "temp:Crystal_SMD_SeikoEpson_FC255-2pin_4.9x1.8mm" H 6000 6800 50  0001 C CNN
F 3 "~" H 6000 6800 50  0001 C CNN
	1    6000 6800
	1    0    0    -1  
$EndComp
Text Label 5650 6800 2    50   ~ 0
LSE_IN
Text Label 6350 6800 0    50   ~ 0
LSE_OUT
$Comp
L Device:C_Small C13
U 1 1 5B8B10AC
P 5750 7000
F 0 "C13" H 5650 7050 50  0000 R CNN
F 1 "12p" H 5650 6950 50  0000 R CNN
F 2 "Capacitors_SMD:C_0603" H 5750 7000 50  0001 C CNN
F 3 "~" H 5750 7000 50  0001 C CNN
	1    5750 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C14
U 1 1 5B8B1134
P 6250 7000
F 0 "C14" H 6350 7050 50  0000 L CNN
F 1 "12p" H 6350 6950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6250 7000 50  0001 C CNN
F 3 "~" H 6250 7000 50  0001 C CNN
	1    6250 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0142
U 1 1 5B8B1664
P 5750 7100
F 0 "#PWR0142" H 5750 6850 50  0001 C CNN
F 1 "GND" H 5755 6927 50  0000 C CNN
F 2 "" H 5750 7100 50  0001 C CNN
F 3 "" H 5750 7100 50  0001 C CNN
	1    5750 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 5B8B16D0
P 6250 7100
F 0 "#PWR0143" H 6250 6850 50  0001 C CNN
F 1 "GND" H 6255 6927 50  0000 C CNN
F 2 "" H 6250 7100 50  0001 C CNN
F 3 "" H 6250 7100 50  0001 C CNN
	1    6250 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6900 6250 6800
Wire Wire Line
	6250 6800 6350 6800
Wire Wire Line
	6250 6800 6150 6800
Connection ~ 6250 6800
Wire Wire Line
	5850 6800 5750 6800
Wire Wire Line
	5750 6800 5750 6900
Connection ~ 5750 6800
Wire Wire Line
	5750 6800 5650 6800
$Comp
L Connector:Conn_01x04_Male J5
U 1 1 5B9102BA
P 10600 1700
F 0 "J5" H 10573 1673 50  0000 R CNN
F 1 "RS485" H 10573 1582 50  0000 R CNN
F 2 "KUT_Connector:DF3A-4P-2DSA" H 10600 1700 50  0001 C CNN
F 3 "~" H 10600 1700 50  0001 C CNN
	1    10600 1700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10300 2000 10300 1700
Wire Wire Line
	10300 1700 10400 1700
Wire Wire Line
	10300 1500 10300 1600
Wire Wire Line
	10300 1600 10400 1600
$Comp
L Connector:Conn_01x05_Male J4
U 1 1 5B9225BB
P 700 3000
F 0 "J4" H 806 3378 50  0000 C CNN
F 1 "GNSS" H 806 3287 50  0000 C CNN
F 2 "KUT_Connector:DF3A-5P-2DSA" H 700 3000 50  0001 C CNN
F 3 "~" H 700 3000 50  0001 C CNN
	1    700  3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2700 1000 2800
Wire Wire Line
	1000 2800 900  2800
Wire Wire Line
	900  2900 1000 2900
Wire Wire Line
	1000 2900 1000 3300
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5B98F58F
P 700 1300
F 0 "J1" H 806 1478 50  0000 C CNN
F 1 "PWRIN" H 806 1387 50  0000 C CNN
F 2 "KUT_Connector:DF3A-2P-2DSA" H 700 1300 50  0001 C CNN
F 3 "~" H 700 1300 50  0001 C CNN
	1    700  1300
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5B999DAE
P 1100 1300
F 0 "#FLG0101" H 1100 1375 50  0001 C CNN
F 1 "PWR_FLAG" V 1100 1427 30  0000 L CNN
F 2 "" H 1100 1300 50  0001 C CNN
F 3 "~" H 1100 1300 50  0001 C CNN
	1    1100 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 1300 1000 1300
Connection ~ 1000 1300
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5B9A3054
P 1100 1400
F 0 "#FLG0102" H 1100 1475 50  0001 C CNN
F 1 "PWR_FLAG" V 1100 1527 30  0000 L CNN
F 2 "" H 1100 1400 50  0001 C CNN
F 3 "~" H 1100 1400 50  0001 C CNN
	1    1100 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 1400 1000 1400
Connection ~ 1000 1400
Text Label 8600 1650 2    50   ~ 0
RS485_RX
Wire Wire Line
	8800 1650 8600 1650
Wire Wire Line
	8600 2050 8800 2050
Text Label 8600 1850 2    50   ~ 0
RS485_EN
Wire Wire Line
	8600 1850 8700 1850
Connection ~ 8700 1850
Wire Wire Line
	8700 1850 8700 1950
$Comp
L Jumper:SolderJumper_2_Bridged JP1
U 1 1 5B9D2040
P 8700 1400
F 0 "JP1" V 8656 1468 50  0000 L CNN
F 1 "~HalfDuplex" V 8746 1468 40  0000 L CNN
F 2 "temp:GS2-Bridged" H 8700 1400 50  0001 C CNN
F 3 "~" H 8700 1400 50  0001 C CNN
	1    8700 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 1550 8700 1750
Connection ~ 8700 1750
Wire Wire Line
	900  3900 1000 3900
Text Label 1100 4000 0    50   ~ 0
RESET
Wire Wire Line
	1100 4200 900  4200
Wire Wire Line
	900  4400 1100 4400
Wire Wire Line
	1000 4500 1000 3900
$Comp
L temp:SW_Push_Tact_4Pin SW1
U 1 1 5BA9BB17
P 4250 2800
F 0 "SW1" V 4500 2700 50  0000 C CNN
F 1 "SW_RESET" V 4600 2700 50  0000 C CNN
F 2 "temp:SW_SPST_SKRPACE010" H 4250 3000 50  0001 C CNN
F 3 "" H 4250 3000 50  0001 C CNN
	1    4250 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 2500 4250 2500
Connection ~ 4450 2500
NoConn ~ 4250 3000
Wire Wire Line
	4250 2500 4250 2600
Wire Wire Line
	4050 2600 4050 2500
Wire Wire Line
	4050 2500 3850 2500
Wire Wire Line
	4050 3000 4050 3100
Wire Wire Line
	4050 3100 3850 3100
Wire Wire Line
	3850 2500 3850 3100
Wire Wire Line
	3850 3100 3850 3200
Connection ~ 3850 3100
Text Label 6550 4050 0    50   ~ 0
RS485_EN
$Comp
L Device:R_Small R14
U 1 1 5B5D4C6B
P 1250 6650
F 0 "R14" H 1309 6696 50  0000 L CNN
F 1 "1k" H 1309 6605 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 1250 6650 50  0001 C CNN
F 3 "~" H 1250 6650 50  0001 C CNN
	1    1250 6650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R15
U 1 1 5B5E8AA9
P 1450 6650
F 0 "R15" H 1509 6696 50  0000 L CNN
F 1 "1k" H 1509 6605 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 1450 6650 50  0001 C CNN
F 3 "~" H 1450 6650 50  0001 C CNN
	1    1450 6650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 5B5E8B21
P 1450 6550
F 0 "#PWR02" H 1450 6400 50  0001 C CNN
F 1 "+3.3V" H 1465 6723 50  0000 C CNN
F 2 "" H 1450 6550 50  0001 C CNN
F 3 "" H 1450 6550 50  0001 C CNN
	1    1450 6550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5B5E8B8C
P 1250 6550
F 0 "#PWR01" H 1250 6400 50  0001 C CNN
F 1 "+3.3V" H 1265 6723 50  0000 C CNN
F 2 "" H 1250 6550 50  0001 C CNN
F 3 "" H 1250 6550 50  0001 C CNN
	1    1250 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6750 1450 6850
Wire Wire Line
	1450 6850 1750 6850
Wire Wire Line
	1250 6750 1250 6950
Wire Wire Line
	1250 6950 1750 6950
Wire Wire Line
	1150 6850 1450 6850
Connection ~ 1450 6850
Wire Wire Line
	1250 6950 1150 6950
Connection ~ 1250 6950
$Comp
L Connector:Conn_01x05_Male J6
U 1 1 5B627160
P 10200 4000
F 0 "J6" H 10172 4023 50  0000 R CNN
F 1 "ReleaseSys" H 10172 3932 50  0000 R CNN
F 2 "KUT_Connector:DF3A-5P-2DSA" H 10200 4000 50  0001 C CNN
F 3 "~" H 10200 4000 50  0001 C CNN
	1    10200 4000
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR03
U 1 1 5B627275
P 9900 3700
F 0 "#PWR03" H 9900 3550 50  0001 C CNN
F 1 "+3.3V" H 9915 3873 50  0000 C CNN
F 2 "" H 9900 3700 50  0001 C CNN
F 3 "" H 9900 3700 50  0001 C CNN
	1    9900 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3700 9900 3800
Wire Wire Line
	9900 3800 10000 3800
$Comp
L power:GND #PWR04
U 1 1 5B6320CB
P 9900 4300
F 0 "#PWR04" H 9900 4050 50  0001 C CNN
F 1 "GND" H 9905 4127 50  0000 C CNN
F 2 "" H 9900 4300 50  0001 C CNN
F 3 "" H 9900 4300 50  0001 C CNN
	1    9900 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3900 10000 3900
Wire Wire Line
	9900 3900 9900 4300
Text Label 9800 4000 2    50   ~ 0
TRIGGER
Wire Wire Line
	9800 4000 10000 4000
Text Label 9800 4100 2    50   ~ 0
BUZZER
Wire Wire Line
	9800 4100 10000 4100
Text Label 9800 4200 2    50   ~ 0
FLIGHTPIN
Wire Wire Line
	9800 4200 10000 4200
NoConn ~ 6550 2850
NoConn ~ 5250 4250
NoConn ~ 5250 4350
NoConn ~ 6550 3850
$Comp
L MCU_ST_STM32L4:STM32L431RBTx U1
U 1 1 5B543CE1
P 5950 3750
F 0 "U1" H 6150 2000 50  0000 C CNN
F 1 "STM32L431RBTx" H 6400 1900 50  0000 C CNN
F 2 "Housings_QFP:LQFP-64_10x10mm_Pitch0.5mm" H 5350 2050 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00257211.pdf" H 5950 3750 50  0001 C CNN
	1    5950 3750
	1    0    0    -1  
$EndComp
Text Label 6550 3250 0    50   ~ 0
TRIGGER
Text Label 6550 3150 0    50   ~ 0
BUZZER
Text Label 6550 3050 0    50   ~ 0
FLIGHTPIN
Wire Wire Line
	10000 2850 9750 2850
Wire Wire Line
	10600 2850 10750 2850
Wire Wire Line
	10400 2850 10300 2850
Wire Wire Line
	10600 3050 10750 3050
Wire Wire Line
	10300 3050 10400 3050
Wire Wire Line
	10000 3050 9750 3050
$EndSCHEMATC
