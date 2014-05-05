Alternative Open Source Firmware for HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to Homematic HM\-LC\-Sw1PBU\-FM as Custom Open Source Firmware speaking BidCos.

Ported Askin Library from FHEM Forum to Atmel ATmega 644A (https://github.com/trilu2000/AskSinLib). Runs with Arduino.

Hardware:
* CC1100 on 868,3 MHz speaking BidCos
* Atmel ATmega 644A (http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A\_PA-324A\_PA-644A\_PA-1284\_P\_datasheet.pdf)
* Controller runs at 3,3V at 8MHz. According to the datasheet up to 5.5V is safe.
* Relay
* Current Sensor: MP13 + MP29 (PA0/ADC0 at ATmega) Shunt amplified by TLC272 OpAmp (http://www.ti.com/lit/ds/symlink/tlc272.pdf)
* Fuses:
```
 avrdude: Device signature = 0x1e9609
 avrdude: safemode: lfuse reads as FF
 avrdude: safemode: hfuse reads as DA
 avrdude: safemode: efuse reads as FD
```

To flash the device you need to solder the colored wires to the ISP port. For debugging you may want the thin wires to the UART port:
![](https://raw.github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM/master/hardware/images/isp.jpg "HM-LC-Sw1PBU-FM with connected UART (thin wires) and ISP (colored wired)")

More images can be found in the hardware/images/ subfolder.

Warnings:
* DO NOT CONNECT THE BOARD to 230V. This will be dangerous, since there is no voltage regulator! All ports including uart and SPI may have 230V potential to ground.
* YOU DO THIS ON YOUR OWN RISK
* This will probably void your warrenty. The device most probably will loose its CE certification.
* There is no way back to the original firmware since we do not have any copy of it

Instructions Hardware:
* Connect your ISP Programmer to the top board of the HM-LC-Sw1PBU-FM to the following testpoints:
  * MP2 - 3,3V 
  * MP3 - Reset
  * MP4 - MOSI
  * MP5 - MISO
  * MP6 - CLK
  * MP15 - GND
* Connect your UART to the following test points (optional for debugging):
  * MP9 - RX
  * MP10 - TX
  * MP16 - GND
* Power the controller using your programmer. Both 3,3V (like Raspberry Pi without any mods) and 5V Programmers (like most USB programmers) work perfectly
* The relay and current sense will not work while testing with 3,3 or 5V. Everything else will

Instructions Software:
* Install jabduino (https://github.com/jabdoa2/jabduino) into your Arduino hardware folder (important: there should be no minus in the foldername)
    * Linux: /usr/share/arduino/hardware 
    * Windows: folder "hardware" in your Arduino directory
- Checkout to sketchbook/Asksin\_HM\_LC\_Sw1PBU\_FM (important: there should be no minus in the foldername)
* Open Arduino
* Open Sketch "Asksin\_HM\_LC\_Sw1PBU\_FM"
* Select Tools->Board->"Jabduino ATmega644A"
* Change HMID in Register.h to the original HMID of your device
* Compile and Upload with programmer (did not try the arduino bootloader)

Features ([x] working [p] partial/not finished [ ] not working):
- [x] Pairing of central via Register.h
- [x] Pairing of central via Configbutton (long press)
- [x] Reset device (double long press)
- [x] getConfig Device
- [x] regSet Device in FHEM
- [x] Two remote channels for both buttons (channel 1 and 2)
- [x] Peering of button via Register.h
- [x] Peering of button via peerChan in FHEM
- [x] getConfig button in FHEM
- [x] regSet button in FHEM
- [x] Controlling peered devices via button press (original homematic devices do work perfectly)
- [x] Actor channel for internal relay (channel 3)
- [x] Peering of actor via Register.h
- [x] Peering of actor via peerChan in FHEM
- [x] getConfig actor in FHEM
- [x] regSet actor in FHEM
- [x] set on/set off in FHEM
- [x] toogle in FHEM
- [x] controlling actor via peered devices
- [x] Showing current status in FHEM (Copy device config below)
- [x] Reading current sensor and sending it via RF (message type 5E. Same format as HM-ES-PMSw1-Pl).
- [x] Showing current sensor value in FHEM. (Copy device config below).
- [x] Virtual Actor channel for double-throw switch/Wechselschalter (channel 4)
- [x] Set actor channel depending on current input
- [x] Toggeling relay (channel 3) when toggeling virtual channel
- [ ] Interpreting current sensor values (60W ~= 6k/5W LED ~= 3k)
- [ ] Sending 41 Messages with current value to other actors
- [x] Load defaults to registers when peering (for both actor, remote, single and dual peerings)

Using device in FHEM:
Copy fhem/99\_Asksin\_HM\_LC\_Sw1PBU\_FM\_CustomFW.pm to FHEM/ in your FHEM installation and restart.
