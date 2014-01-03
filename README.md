Asksin\_HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to Homematic HM\-LC\-Sw1PBU\-FM

Ported Askin Library from FHEM Forum to Atmel ATmega 644A. Runs with Arduino.

Hardware:
* CC1100
* Atmel ATmega 644A
* Relay
* Currency Sensor

To flash the device you need to solder the colored wires to the ISP port. For debugging you need the thin wires to the UART port:
![](https://raw.github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM/master/hardware/images/isp.jpg "HM-LC-Sw1PBU-FM with connected UART (thin wires) and ISP (colored wired)")

More images can be found in the hardware/images/ subfolder.

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

Tested/Working features:
- [X] Pairing of central via Register.h
- [ ] Pairing of central via Configbutton
- [ ] getConfig Device
- [ ] regSet Device in FHEM (untested)
- [X] Peering of button via Register.h
- [ ] Peering of button via peerChan in FHEM
- [X] getConfig button in FHEM
- [ ] regSet button in FHEM (untested)
- [X] Controlling peered devices via button press
- [X] Peering of actor via Register.h
- [X] Peering of actor via peerChan in FHEM
- [X] getConfig actor in FHEM
- [ ] regSet actor in FHEM (untested)
- [X] set on/set off in FHEM
- [X] toogle in FHEM
- [X] controlling actor via peered devices
- [ ] Showing current status in FHEM (RF looks correct. May be bug in FHEM device)
