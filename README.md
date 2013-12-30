Asksin\_HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to Homematic HM\-LC\-Sw1PBU\-FM

Ported Askin Library from FHEM Forum to Atmel ATmega 644A. Runs with Arduino.

Hardware:
* CC1100
* Atmel ATmega 644A
* Relay
* Currency Sensor

![Image with connected UART (thin wires) and ISP (colored wired)](https://github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM/blob/master/hardware/images/isp.jpg)

Instructions Hardware:
* WARNING: DO NOT CONNECT THE BOARD to 230V. This will be dangous!
* Only connect 3,3V. 5V may destroy the device.
* The relay will not work while testing. Everything else will
* This will probably void your warrenty
* You do this at your own risk

Connect your ISP Programmer to the top board of the HM-LC-Sw1PBU-FM to the following testpoints:
* MP2 - 3,3V 
* MP3 - Reset
* MP4 - MOSI
* MP5 - MISO
* MP6 - CLK
* MP15 - GND

UART is also at the top board at those testpoints:
* MP9 - RX
* MP10 - TX
* MP16 - GND

Other intersting ports:
* MP13 + MP29 - Currency Sensor (PA0/ADC0 at ATmega). Not implemented in original Firmware. Need to be explored

Instructions Software:
* Install jabduino (https://github.com/jabdoa2/jabduino) into your Arduino hardware folder
    * Linux: /usr/share/arduino/hardware 
    * Windows: folder "hardware" in your Arduino directory
* Open Arduino
* Open Sketch "HM\_LC\_Sw1PBU\_FM"
* Select Tools->Board->"Jabduino ATmega644A"
* Compile and Upload with programmer (did not try the arduino bootloader)
