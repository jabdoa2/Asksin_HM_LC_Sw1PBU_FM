//- load library's --------------------------------------------------------------------------------------------------------
#include "AskSin.h"

//- serial communication --------------------------------------------------------------------------------------------------
const char helptext1[] PROGMEM = {												// help text for serial console
	"\n"
	"Available commands:" "\n"
	"  p                - start pairing with master" "\n"
	"  b[0]  b[n]  s    - send a string, b[0] is length (50 bytes max)" "\n"
	"\n"
	"  i[0]. i[1]. e    - show eeprom content, i[0]. start address, i[1]. length" "\n"
	"  i[0]. b[1]  f    - write content to eeprom, i[0]. address, i[1] byte" "\n"
	"  c                - clear eeprom complete, write 0 from start to end" "\n"
	"\n"
	"  b[c]  b[l]  b    - send button event, b[c] channel, b[l] short 0 or long 1" "\n"
	"  a                - stay awake for TRX module (valid if power mode = 2)" "\n"
	"  t                - gives an overview of the device configuration" "\n"
	"\n"
	"  $nn for HEX input (e.g. $AB,$AC ); b[] = byte, i[]. = integer " "\n"
};
InputParser::Commands cmdTab[] PROGMEM = {
	{ 'h', 0, showHelp },
	{ 'p', 0, sendPairing },
	{ 's', 1, sendCmdStr },
	{ 'e', 0, showEEprom },
	{ 'f', 2, writeEEprom },
	{ 'c', 0, clearEEprom },
	{ 't', 0, testConfig },

	{ 'b', 1, buttonSend },
	{ 'a', 0, stayAwake },
	
	{ 'r', 0, resetDevice },
	{ 0 }    
};
InputParser parser (50, cmdTab);


//- homematic communication -----------------------------------------------------------------------------------------------
s_jumptable jumptable[] PROGMEM = {												// jump table for HM communication
	{ 0x01, 0x0E, HM_Status_Request },
	{ 0x11, 0x02, HM_Set_Cmd },
	{ 0x11, 0x04, HM_Reset_Cmd },
	{ 0x3E, 0x00, HM_Switch_Event  },
	{ 0x40, 0x00, HM_Remote_Event  },
	{ 0xFF, 0xFF, HM_Config_Changed },
	{ 0x0 }
};
HM hm (jumptable, regMcPtr);													// declare class for handling HM communication
BK bk[3];																		// declare 1 inxtStatnces of the button key handler
RL rl[1];																		// declare one inxtStatnce of relay class

//- some timer test
uint32_t nTimer;

//- main functions --------------------------------------------------------------------------------------------------------
void setup() {
	Serial.begin(57600);														// starting serial messages

	// some power savings
	ADCSRA = 0;																	// disable ADC
	power_all_disable();														// all devices off
	power_timer0_enable();														// we need timer0 for delay function
	//power_timer2_enable();													// we need timer2 for PWM
	power_usart0_enable();														// it's the serial console
	//power_twi_enable();														// i2c interface, not needed yet
	power_spi_enable();															// enables SPI master
	//power_adc_enable();

	// init HM module
	hm.init();																	// initialize HM module
	hm.ld.config(0);															// configure the status led pin
	hm.setPowerMode(0);															// power mode for HM device
	hm.setConfigEvent();														// reread config 

	// configure some buttons - config(tIdx, tPin, tTimeOutShortDbl, tLongKeyTime, tTimeOutLongDdbl, tCallBack)
	bk[0].config(0,15,0,5000,5000,buttonState);									// button 0 for channel 0 for send pairing string, and double press for reseting device config
	bk[1].config(1,14,0,1000,5000,buttonState); // channel 1 to 2 as push button
	bk[2].config(2,8,0,1000,5000,buttonState);

	// init relay stuff
	rl[0].config(3,0,12,0,0,0);		// Channel 3 as aktor											// configure the relay to monostable, on pin 3
	rl[0].setCallBack(&relayState,&hm,1,1);
	
	// show help screen and config
	showHelp();																	// shows help screen on serial console
	showSettings();																// show device settings

      Serial << F("version 020") << '\n';															// show device settings
}

void loop() {
	// poll functions for serial console, HM module, button key handler and relay handler
	parser.poll();																// handle serial input from console
	hm.poll();																	// HOMEMATIC task scheduler
	bk->poll();																	// key handler poll
	rl->poll();																	// relay handler poll
	
	//if (nTimer < millis()) {
	//	nTimer = millis() + 30000;												// jump in every 30 seconds
	//	Serial << "t:" << (millis()/1000) << '\n';								// time stamp
	//	hm.stayAwake(100);														// wait 100ms to get the output in console
	//}
}


//- key handler functions -------------------------------------------------------------------------------------------------
void buttonState(uint8_t idx, uint8_t state) {
	// possible events of this function:
	//   0 - short key press
	//   1 - double short key press
	//   2 - long key press
	//   3 - repeated long key press
	//   4 - end of long key press
	//   5 - double long key press
	//   6 - time out for a double long

	Serial << "i:" << idx << ", s:" << state << '\n';							// some debug message

	// channel device
	if (idx == 0) {
		if (state == 0) hm.startPairing();										// short key press, send pairing string
		if (state == 2) hm.ld.set(2);											// long key press could mean, you like to go for reseting the device
		if (state == 6) hm.ld.set(0);											// time out for double long, stop slow blinking
		if (state == 5) hm.reset();												// double long key press, reset the device
	}

	// channel 1 - 2
	if ((idx >= 1) && (idx <= 2)) {
		if ((state == 0) || (state == 1)) hm.sendPeerREMOTE(idx,0,0);			// short key or double short key press detected
		if ((state == 2) || (state == 3)) hm.sendPeerREMOTE(idx,1,0);			// long or repeated long key press detected
		if (state == 4) hm.sendPeerREMOTE(idx,2,0);								// end of long or repeated long key press detected
	}
}

//- relay handler functions -----------------------------------------------------------------------------------------------
void relayState(uint8_t cnl, uint8_t curStat, uint8_t nxtStat) {
	Serial << "c:" << cnl << " cS:" << curStat << " nS:" << nxtStat << '\n';	// some debug message
}


//- HM functions ----------------------------------------------------------------------------------------------------------
void HM_Status_Request(uint8_t cnl, uint8_t *data, uint8_t len) {
	// message from master to client while requesting the channel specific status
	// client has to send an INFO_ACTUATOR_MESSAGE with the current status of the requested channel
	// there is no payload; data[0] could be ignored

	//Serial << F("\nxtStattus_Request; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
	if (cnl == 3) rl[0].sendStatus();											// send the current status
}
void HM_Set_Cmd(uint8_t cnl, uint8_t *data, uint8_t len) {
	// message from master to client for setting a defined value to client channel
	// client has to send an ACK with the current status; payload is typical 3 bytes
	// data[0] = status message; data[1] = down,up,low battery; data[2] = rssi (signal quality)

	//Serial << F("\nSet_Cmd; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
	if (cnl == 3) rl[0].trigger11(data[0], data+1, (len>4)?data+3:NULL);
}
void HM_Reset_Cmd(uint8_t cnl, uint8_t *data, uint8_t len) {
	//Serial << F("\nReset_Cmd; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
	hm.send_ACK();																// send an ACK
	if (cnl == 0) hm.reset();													// do a reset only if channel is 0
}
void HM_Switch_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
	// sample needed!
	// ACK is requested but will send automatically
	Serial << F("\nSwitch_Event; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
}

void HM_Remote_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
	// message from a remote to the client device; this event pop's up if the remote is peered
	// cnl = indicates client device channel
	// data[0] the remote channel, but also the information for long key press - ((data[0] & 0x40)>>6) extracts the long key press
	// data[1] = typically the key counter of the remote
	
	//Serial << F("\nRemote_Event; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
	if (cnl == 3) rl[0].trigger40(((data[0] & 0x40)>>6),data[1],(void*)&regMC.ch3.l3);
}
void HM_Sensor_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
	// sample needed!
	// ACK is requested but will send automatically
	Serial << F("\nSensor_Event; cnl: ") << pHex(cnl) << F(", data: ") << pHex(data,len) << "\n\n";
}
void HM_Config_Changed(uint8_t cnl, uint8_t *data, uint8_t len) {
	Serial << F("config changed\n");


}


//- config functions ------------------------------------------------------------------------------------------------------
void sendCmdStr() {																// reads a sndStr from console and put it in the send queue
 	memcpy(hm.send.data,parser.buffer,parser.count());							// take over the parsed byte data
	Serial << F("s: ") << pHexL(hm.send.data, hm.send.data[0]+1) << '\n';		// some debug string
	hm.send_out();																// fire to send routine
}
void sendPairing() {															// send the first pairing request
	hm.startPairing();
}

void showEEprom() {
	uint16_t start, len;
	uint8_t buf[32];
	
	parser >> start >> len;
	if (len == 0) len = E2END - start;
	
	Serial << F("EEPROM listing, start: ") << start << F(", len: ") << len << '\n';

	for (uint16_t i = start; i < len; i+=32) {
		eeprom_read_block(buf,(void*)i,32);
		Serial << pHex(i>>8) << pHex(i&0xFF) << F("   ") << pHex(buf,32) << '\n';
	}
}
void writeEEprom() {
	uint16_t addr;
	uint8_t data;
	
	for (uint8_t i = 0; i < parser.count(); i+=3) {
		parser >> addr >> data;
		eeprom_write_byte((uint8_t*)addr,data);
		Serial << F("Write EEprom, Address: ") << pHex(addr>>8) << pHex(addr&0xFF) << F(", Data: ") << pHex(data) << '\n';
	}
}
void clearEEprom() {															// clear settings
	Serial << F("Clear EEprom, size: ") << E2END+1 << F(" bytes") << '\n';
	for (uint16_t i = 0; i <= E2END; i++) {
		eeprom_write_byte((uint8_t*)i,0);
	}
	Serial << F("done") << '\n';
}

void showHelp() {																// display help on serial console
	showPGMText(helptext1);
}
void showSettings() {															// shows device settings on serial console
	hm.printSettings();															// print settings of own HM device
	Serial << F("FreeMem: ") << freeMemory() << F(" byte's\n");					// displays the free memory
}

void testConfig() {																// shows the complete configuration of slice table and peer database
	hm.printConfig();															// prints register and peer config
}

void buttonSend() {
	uint8_t cnl, lpr;
	parser >> cnl >> lpr;
	
	Serial << "button press, cnl: " << cnl << ", long press: " << lpr << '\n';	// some debug message
	hm.sendPeerREMOTE(cnl,lpr,0);												// parameter: button/channel, long press, battery
}
void stayAwake() {
	hm.stayAwake(30000);														// stay awake for 30 seconds
}
void resetDevice() {
	Serial << F("reset device, clear eeprom...\n");
	hm.reset();
	Serial << F("reset done\n");
}
