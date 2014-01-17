package main;

use strict;
use warnings;
use POSIX;
use Switch;

sub
Asksin_HM_LC_Sw1PBU_FM_CustomFW_Initialize($$)
{
  my ($hash) = @_;
}

sub
registerHM_LC_Sw1PBU_FM_CustomFW()
{
  {$HMConfig::culHmModel{"F0A9"} = {name=>"HM-LC-Sw1PBU-FM-CustomFW",st=>'remoteAndSwitch',cyc=>'',rxt=>'',lst=>'1,3:3p,4:1p.2p',chn=>"Btn:1:2,Sw:3:3"}}
  {$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW01"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
  {$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW02"} = $HMConfig::culHmSubTypeSets{"THSensor"}};
  {$HMConfig::culHmChanSets{"HM-LC-Sw1PBU-FM-CustomFW03"} = $HMConfig::culHmSubTypeSets{"switch"}};
  {$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW01"}  = $HMConfig::culHmRegType{remote}};
  {$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW02"}  = $HMConfig::culHmRegType{remote}};
  {$HMConfig::culHmRegChan{"HM-LC-Sw1PBU-FM-CustomFW03"}  = $HMConfig::culHmRegType{switch}};
  #Log(1, "Registered F0A9");
}

#Log(1, "Loaded CustomFS");
InternalTimer(gettimeofday()+10,"registerHM_LC_Sw1PBU_FM_CustomFW","nothing", 0);

sub CUL_HM_ParseremoteAndSwitch($$$$$$) {
  my($mFlg,$mTp,$src,$dst,$p,$target) = @_;
  my @entities;
  my @event;

#  Log 1,"General  entering with $mFlg,$mTp,$src,$dst,$p";

  if (($mTp eq "02" && $p =~ m/^01/) ||  # handle Ack_Status
      ($mTp eq "10" && $p =~ m/^06/)) { #    or Info_Status message here


    my $rSUpdt = 0;# require status update
    my ($subType,$chn,$val,$err) = ($1,hex($2),hex($3)/2,hex($4))
                        if($p =~ m/^(..)(..)(..)(..)/);
    $chn = sprintf("%02X",$chn&0x3f);
    my $chId = $src.$chn;
    my $shash = $modules{CUL_HM}{defptr}{$chId}
                           if($modules{CUL_HM}{defptr}{$chId});

    my $vs = ($val==100 ? "on":($val==0 ? "off":"$val %")); # user string...

    push @event,"level:$val %";
    push @event,"pct:$val"; # duplicate to level - necessary for "slider"
    push @event,"deviceMsg:$vs$target" if($chn ne "00");
    push @event,"state:".$vs;
    my $action; #determine action
    push @event, "timedOn:".(($err&0x40)?"running":"off");
    push @entities,CUL_HM_UpdtReadBulk($shash,1,@event);

  }
  elsif ($mTp eq "5E" ||$mTp eq "5F" ) {  #    POWER_EVENT_CYCLIC

    my $shash = $modules{CUL_HM}{defptr}{$src."03"}
                             if($modules{CUL_HM}{defptr}{$src."03"});
    my ($eCnt,$P,$I,$U,$F) = unpack 'A6A6A4A4A2',$p;
#    push @event, "energy:"   .(hex($eCnt)&0x7fffff)/10;# 0.0  ..838860.7  Wh
#    push @event, "power:"    . hex($P   )/100;         # 0.0  ..167772.15 W
    push @event, "current:"  . hex($I   )/1;           # 0.0  ..65535.0   mA
#    push @event, "voltage:"  . hex($U   )/10;          # 0.0  ..6553.5    mV
#    push @event, "frequency:".(hex($F   )/100+50);      # 48.72..51.27     Hz
#    push @event, "boot:"     .((hex($eCnt)&0x800000)?"on":"off");
    push @entities,CUL_HM_UpdtReadBulk($shash,1,@event);
  } else {

  }

  return @entities;
}


1;
