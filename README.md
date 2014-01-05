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
- [x] Pairing of central via Register.h
- [ ] Pairing of central via Configbutton
- [ ] getConfig Device (not working)
- [ ] regSet Device in FHEM (working only on channels)
- [x] Peering of button via Register.h
- [x] Peering of button via peerChan in FHEM
- [x] getConfig button in FHEM
- [x] regSet button in FHEM
- [x] Controlling peered devices via button press
- [x] Peering of actor via Register.h
- [x] Peering of actor via peerChan in FHEM
- [x] getConfig actor in FHEM
- [x] regSet actor in FHEM
- [x] set on/set off in FHEM
- [x] toogle in FHEM
- [x] controlling actor via peered devices
- [ ] Showing current status in FHEM (RF looks correct. May be bug in FHEM device)

Using device in FHEM:

With current FHEM version you just need to paste the following code (in the command field on top).

```
{$HMConfig::culHmModel{"F0A9"} = {name=>"HM-LC-Sw1PBU-FM-CustomFW",st=>'remoteAndSwitch',cyc=>'',rxt=>'',lst=>'1,4',chn=>"Btn:1:2,Sw:3:3"}};
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW01"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW02"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW03"} = $HMConfig::culHmSubTypeSets{"switch"}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW01"}  = $HMConfig::culHmRegType{remote}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW02"}  = $HMConfig::culHmRegType{remote}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW03"}  = $HMConfig::culHmRegType{switch}};
```
