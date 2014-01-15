Alternative Open Source Firmware for HM\_LC\_Sw1PBU\_FM
======================

Asksin Library ported to Homematic HM\-LC\-Sw1PBU\-FM as Custom Open Source Firmware speaking BidCos.

Ported Askin Library from FHEM Forum to Atmel ATmega 644A (https://github.com/trilu2000/AskSinLib). Runs with Arduino.

Hardware:
* CC1100 on 868,3 MHz speaking BidCos
* Atmel ATmega 644A (http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf)
* Relay
* Current Sensor: Shunt amplified by TLC272 OpAmp (http://www.ti.com/lit/ds/symlink/tlc272.pdf)

To flash the device you need to solder the colored wires to the ISP port. For debugging you need the thin wires to the UART port:
![](https://raw.github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM/master/hardware/images/isp.jpg "HM-LC-Sw1PBU-FM with connected UART (thin wires) and ISP (colored wired)")

More images can be found in the hardware/images/ subfolder.

Instructions Hardware:
* WARNING: DO NOT CONNECT THE BOARD to 230V. This will be dangous, since there is no voltage regulator! All ports including uart and SPI may have 230V potential to ground.
* Controller runs at 3,3V at 8MHz. According to the datasheet up to 5.5V should be save (untested).
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
* MP13 + MP29 - Current Sensor (PA0/ADC0 at ATmega). Not implemented in original Firmware. Need to be explored

Instructions Software:
* Install jabduino (https://github.com/jabdoa2/jabduino) into your Arduino hardware folder
    * Linux: /usr/share/arduino/hardware 
    * Windows: folder "hardware" in your Arduino directory
* Open Arduino
* Open Sketch "Asksin\_HM\_LC\_Sw1PBU\_FM"
* Select Tools->Board->"Jabduino ATmega644A"
* Compile and Upload with programmer (did not try the arduino bootloader)

Tested/Working features:
- [x] Pairing of central via Register.h
- [x] Pairing of central via Configbutton
- [x] getConfig Device
- [x] regSet Device in FHEM
- [x] Peering of button via Register.h
- [x] Peering of button via peerChan in FHEM
- [x] getConfig button in FHEM
- [x] regSet button in FHEM
- [x] Controlling peered devices via button press (original homematic devices do work perfectly)
- [x] Peering of actor via Register.h
- [x] Peering of actor via peerChan in FHEM
- [x] getConfig actor in FHEM
- [x] regSet actor in FHEM
- [x] set on/set off in FHEM
- [x] toogle in FHEM
- [x] controlling actor via peered devices
- [x] Showing current status in FHEM (Working with patch below. Will hopefully go upstream soon)
- [x] Reading Current sensor and sending it via RF (message type 5E. Same format as HM-ES-PMSw1-Pl)
- [ ] Interpreting current sensor values (10 bit value. what does 1024 mean? 10A?)
- [ ] Controlling actor by current sensor
- [ ] Sending remote event by current sensor

Using device in FHEM:

With current FHEM version you just need to paste the following code (in the command field on top).

```
{$HMConfig::culHmModel{"F0A9"} = {name=>"HM-LC-Sw1PBU-FM-CustomFW",st=>'remoteAndSwitch',cyc=>'',rxt=>'',lst=>'1,3:3p,4:1p.2p',chn=>"Btn:1:2,Sw:3:3"}}
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW01"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW02"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
{$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW03"} = $HMConfig::culHmSubTypeSets{"switch"}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW01"}  = $HMConfig::culHmRegType{remote}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW02"}  = $HMConfig::culHmRegType{remote}};
{$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW03"}  = $HMConfig::culHmRegType{switch}};
```

To see the current status of the actor and the read the current sensor you need to patch your 10_CUL_HM.pm (line 1071):
1. Change:
```
  elsif($st =~ m /^(switch|dimmer|blindActuator|remoteAndSwitch)$/) {##########################
```
To:
```
  elsif($st =~ m /^(switch|dimmer|blindActuator)$/) {##########################
```

2. After (line 1176):
```
         push @event,"battery:" . (($err&0x80) ? "low" : "ok" );
       }
     }
```
Add the following:
```
    elsif ($mTp eq "5E" ||$mTp eq "5F" ) {  #    POWER_EVENT_CYCLIC
      $shash = $modules{CUL_HM}{defptr}{$src."03"}
                             if($modules{CUL_HM}{defptr}{$src."03"});
      my ($eCnt,$P,$I,$U,$F) = unpack 'A6A6A4A4A2',$p;
#      push @event, "energy:"   .(hex($eCnt)&0x7fffff)/10;# 0.0  ..838860.7  Wh
#      push @event, "power:"    . hex($P   )/100;         # 0.0  ..167772.15 W
      push @event, "current:"  . hex($I   )/1;           # 0.0  ..65535.0   mA
#      push @event, "voltage:"  . hex($U   )/10;          # 0.0  ..6553.5    mV
#      push @event, "frequency:".(hex($F   )/100+50);      # 48.72..51.27     Hz
#      push @event, "boot:"     .((hex($eCnt)&0x800000)?"on":"off");
    }
```

or use the patch in fhem/10_CUL_HM.pm.patch (apply with patch -p1 < 10_CUL_HM.pm.patch). Afterwards restart fhem.
