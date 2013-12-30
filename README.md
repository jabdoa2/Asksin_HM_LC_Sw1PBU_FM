Asksin\_HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to HM\-LC\-Sw1PBU\-FM

Ported Askin Library from FHEM Forum to Atmel Atmega 644A. Runs with Arduino. Use Sanguino as target and change MMCU to atmega644.

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

Serial is also at the top board at those testpoints:
* MP9 - RX
* MP10 - TX
* MP16 - GND


Instructions Software:
* Install jabduino (https://github.com/jabdoa2/jabduino) into your Arduino hardware folder
    * Linux: /usr/share/arduino/hardware 
    * Windows: folder "hardware" in your Arduino directory
* Open Arduino
* Open Sketch "HM\_LC\_Sw1PBU\_FM"
* Select Tools->Board->"Jabduino ATmega644A"
* Compile and Upload with programmer (did not try the arduino bootloader)
