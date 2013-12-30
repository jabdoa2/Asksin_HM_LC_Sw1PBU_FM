Asksin\_HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to HM\-LC\-Sw1PBU\-FM

Ported Askin Library from FHEM Forum to Atmel Atmega 644A. Runs with Arduino. Use Sanguino as target and change MMCU to atmega644.

Instructions:
* Install jabduino (https://github.com/jabdoa2/jabduino) into your Arduino hardware folder
    * Linux: /usr/share/arduino/hardware 
    * Windows: folder "hardware" in your Arduino directory
* Open Arduino
* Open Sketch "HM\_LC\_Sw1PBU\_FM"
* Select Tools->Board->"Jabduino ATmega644A"
* Compile and Upload with programmer (did not try the arduino bootloader)
