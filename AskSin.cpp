//- -----------------------------------------------------------------------------------------------------------------------
// AskSin driver implementation
// 2013-08-03 <horst@diebittners.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// Trx868 documentation https://github.com/ccier/openhm/wiki/Trx868
// Parser sketch from: http://jeelabs.org/2010/10/24/parsing-input-commands/
//- -----------------------------------------------------------------------------------------------------------------------
#include "AskSin.h"

s_pci pci;
s_prl prl;

//- ----------------------------------------------------------------------------------------------------------------------
//- CC1101 communication functions ----------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
	const uint8_t initVal[] PROGMEM = {									// define init settings for TRX868
		0x00, 0x2E,			// IOCFG2: tristate									// non inverted GDO2, high impedance tri state
		0x01, 0x2E,			// IOCFG1: tristate									// low output drive strength, non inverted GD=1, high impedance tri state
		0x02, 0x06,			// IOCFG0: packet CRC ok							// disable temperature sensor, non inverted GDO0, asserts when a sync word has been sent/received, and de-asserts at the end of the packet. in RX, the pin will also de-assert when a package is discarded due to address or maximum length filtering
		0x03, 0x0D,			// FIFOTHR: TX:9 / RX:56							// 0 ADC retention, 0 close in RX, TX FIFO = 9 / RX FIFO = 56 byte
		0x04, 0xE9,			// SYNC1											// Sync word
		0x05, 0xCA,			// SYNC0
		0x06, 0x3D,			// PKTLEN(x): 61									// packet length 61
		0x07, 0x0C,			// PKTCTRL1:										// PQT = 0, CRC auto flush = 1, append status = 1, no address check
		0x0B, 0x06,			// FSCTRL1:											// frequency synthesizer control
		0x0D, 0x21,			// FREQ2
		0x0E, 0x65,			// FREQ1
		0x0F, 0x6A,			// FREQ0
		0x10, 0xC8,			// MDMCFG4
		0x11, 0x93,			// MDMCFG3
		0x12, 0x03,			// MDMCFG2
		0x15, 0x34,			// DEVIATN
		0x16, 0x01,         // MCSM2             
		0x17, 0x30,			// MCSM1: always go into IDLE
		0x18, 0x18,			// MCSM0
		0x19, 0x16,			// FOCCFG
		0x1B, 0x43,			// AGCTRL2
		//0x1E, 0x28,       // ..WOREVT1: tEVENT0 = 50 ms, RX timeout = 390 us
		//0x1F, 0xA0,		// ..WOREVT0: 
		//0x20, 0xFB,		// ..WORCTRL: EVENT1 = 3, WOR_RES = 0
		0x21, 0x56,			// FREND1
		0x25, 0x00,
		0x26, 0x11,			// FSCAL0
		0x2D, 0x35,			// TEST1
		0x3E, 0xC3,			// ?
	};

void CC::init(void) {															// initialize CC1101
	#if defined(CC_DBG)
	Serial << F("CC1101_init: ");
	#endif

	pinMode(SS, OUTPUT);														// set pins for SPI communication
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(SCK, OUTPUT);
	pinMode(GDO0, INPUT);														// config GDO0 as input

	digitalWrite(SS, HIGH);														// SPI init
	digitalWrite(SCK, HIGH);
	digitalWrite(MOSI, LOW);

	SPCR = _BV(SPE) | _BV(MSTR);												// SPI speed = CLK/4

	cc1101_Deselect();															// some deselect and selects to init the TRX868modul
	delayMicroseconds(5);
	cc1101_Select();	
	delayMicroseconds(10);
	cc1101_Deselect();
	delayMicroseconds(41);

	cmdStrobe(CC1101_SRES);														// send reset
        delay(10);

	#if defined(CC_DBG)
	Serial << '1';
	#endif

	for (uint8_t i=0; i<sizeof(initVal); i++) {									// write init value to TRX868
		writeReg(pgm_read_byte(&initVal[i++]), pgm_read_byte(&initVal[i]));	
	}
	
	#if defined(CC_DBG)
	Serial << '2';
	#endif

	cmdStrobe(CC1101_SCAL);														// calibrate frequency synthesizer and turn it off
	while (readReg(CC1101_MARcurStatTE, CC1101_STATUS) != 1) {						// waits until module gets ready
		delayMicroseconds(1);
		#if defined(CC_DBG)
		Serial << '.';
		#endif
	}
	
	#if defined(CC_DBG)
	Serial << '3';
	#endif

	writeReg(CC1101_PATABLE, PA_MaxPower);										// configure PATABLE
	cmdStrobe(CC1101_SRX);														// flush the RX buffer
	cmdStrobe(CC1101_SWORRST);													// reset real time clock

	#if defined(CC_DBG)
	Serial << F(" - ready\n");
	#endif
}
boolean CC::sendData(uint8_t *buf, uint8_t burst) {								// send data packet via RF

	// Going from RX to TX does not work if there was a reception less than 0.5
	// sec ago. Due to CCA? Using IDLE helps to shorten this period(?)
	//ccStrobe(CC1100_SIDLE);
	//uint8_t cnt = 0xff;
	//while(cnt-- && (ccStrobe( CC1100_STX ) & 0x70) != 2)
	//my_delay_us(10);
 	cmdStrobe(CC1101_SIDLE);													// go to idle mode
	cmdStrobe(CC1101_SFRX );													// flush RX buffer
	cmdStrobe(CC1101_SFTX );													// flush TX buffer
	
	//Serial << "tx\n";
	
	if (burst) {																// BURST-bit set?
		cmdStrobe(CC1101_STX  );												// send a burst
		delay(360);																// according to ELV, devices get activated every 300ms, so send burst for 360ms
		//Serial << "send burst\n";
	} else {
		delay(1);																// wait a short time to set TX mode
	}

	writeBurst(CC1101_TXFIFO, buf, buf[0]+1);									// write in TX FIFO

	cmdStrobe(CC1101_SFRX);														// flush the RX buffer
	cmdStrobe(CC1101_STX);														// send a burst

	for(uint8_t i=0; i< 200;++i) {												// after sending out all bytes the chip should go automatically in RX mode
		if( readReg(CC1101_MARcurStatTE, CC1101_STATUS) == MARcurStatTE_RX)
			break;																//now in RX mode, good
		if( readReg(CC1101_MARcurStatTE, CC1101_STATUS) != MARcurStatTE_TX) {
			break;																//neither in RX nor TX, probably some error
		}
		delayMicroseconds(10);
	}

	//uint8_t cnt = 0xff;
	//while(cnt-- && (sendSPI(CC1101_SRX) & 0x70) != 1)
	//delayMicroseconds(10);

	#if defined(CC_DBG)															// some debug message
	Serial << F("<- ") << pHexL(&buf[0], buf[0]+1) << pTime();
	#endif

	//Serial << "rx\n";
	return true;
}
uint8_t CC::receiveData(uint8_t *buf) {											// read data packet from RX FIFO
	uint8_t rxBytes = readReg(CC1101_RXBYTES, CC1101_STATUS);					// how many bytes are in the buffer

	if (rxBytes & 0x7F && !(rxBytes & 0x80)) {									// any byte waiting to be read and no overflow?
		buf[0] = readReg(CC1101_RXFIFO, CC1101_CONFIG);							// read data length
		
		if (buf[0] > CC1101_DATA_LEN)											// if packet is too long
			buf[0] = 0;															// discard packet
		else {
			readBurst(&buf[1], CC1101_RXFIFO, buf[0]);							// read data packet
			readReg(CC1101_RXFIFO, CC1101_CONFIG);								// read RSSI
			
			uint8_t val = readReg(CC1101_RXFIFO, CC1101_CONFIG);				// read LQI and CRC_OK
			trx868.lqi = val & 0x7F;
			trx868.crc_ok = bitRead(val, 7);
		}
	} else buf[0] = 0;															// nothing to do, or overflow

	cmdStrobe(CC1101_SFRX);														// flush Rx FIFO
	cmdStrobe(CC1101_SIDLE);													// enter IDLE state
	cmdStrobe(CC1101_SRX);														// back to RX state
	cmdStrobe(CC1101_SWORRST);													// reset real time clock
//	trx868.rfState = RFSTATE_RX;												// declare to be in Rx state
	
	#if defined(CC_DBG)															// debug message, string should be short, otherwise program stops
	if (buf[0] > 0) Serial << pHexL(&buf[1], buf[0]) << pTime();
	#endif
	
	return buf[0];																// return the data buffer
}
uint8_t CC::detectBurst(void) {													// wake up CC1101 from power down state
	// 10 7/10 5 in front of the received string; 33 after received string
	// 10 - 00001010 - sync word found
	// 7  - 00000111 - GDO0 = 1, GDO2 = 1
	// 5  - 00000101 - GDO0 = 1, GDO2 = 1
	// 33 - 00100001 - GDO0 = 1, preamble quality reached
	// 96 - 01100000 - burst sent
	// 48 - 00110000 - in receive mode
	//
	// Status byte table:
	//	0 current GDO0 value
	//	1 reserved
	//	2 GDO2
	//	3 sync word found
	//	4 channel is clear
	//	5 preamble quality reached
	//	6 carrier sense
	//	7 CRC ok
	//
	// possible solution for finding a burst is to check for bit 6, carrier sense

	// set RXTX module in receive mode
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	cc1101_Deselect();															// deselect CC1101
	cmdStrobe(CC1101_SRX);														// set RX mode again
	delay(3);																	// wait a short time to set RX mode
	
	// todo: check carrier sense for 5ms to avoid wakeup due to normal transmition
	//Serial << "rx\n";
	return bitRead(hm.cc.monitorStatus(),6);									// return the detected signal
}
void CC::setPowerDownxtStatte() {													// put CC1101 into power-down state
	cmdStrobe(CC1101_SIDLE);													// coming from RX state, we need to enter the IDLE state first
	cmdStrobe(CC1101_SFRX);
	cmdStrobe(CC1101_SPWD);														// enter power down state
	//Serial << "pd\n";
}
uint8_t CC::monitorStatus() {
	return readReg(CC1101_PKTSTATUS, CC1101_STATUS);
}

uint8_t CC::sendSPI(uint8_t val) {												// send byte via SPI
	SPDR = val;																	// transfer byte via SPI
	while(!(SPSR & _BV(SPIF)));													// wait until SPI operation is terminated
	return SPDR;
}
void CC::cmdStrobe(uint8_t cmd) {												// send command strobe to the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(cmd);																// send strobe command
	cc1101_Deselect();															// deselect CC1101
}
void CC::writeBurst(uint8_t regAddr, uint8_t *buf, uint8_t len) {				// write multiple registers into the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | WRITE_BURST);												// send register address
	for(uint8_t i=0 ; i<len ; i++) sendSPI(buf[i]);								// send value
	cc1101_Deselect();															// deselect CC1101
}
void CC::readBurst(uint8_t *buf, uint8_t regAddr, uint8_t len) {				// read burst data from CC1101 via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | READ_BURST);												// send register address
	for(uint8_t i=0 ; i<len ; i++) buf[i] = sendSPI(0x00);						// read result byte by byte
	cc1101_Deselect();															// deselect CC1101
}
uint8_t CC::readReg(uint8_t regAddr, uint8_t regType) {							// read CC1101 register via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | regType);													// send register address
	uint8_t val = sendSPI(0x00);												// read result
	cc1101_Deselect();															// deselect CC1101
	return val;
}
void CC::writeReg(uint8_t regAddr, uint8_t val) {								// write single register into the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr);															// send register address
	sendSPI(val);																// send value
	cc1101_Deselect();															// deselect CC1101
}


//- -----------------------------------------------------------------------------------------------------------------------
//- status led functions --------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
void LD::config(uint8_t tPin) {
	pin = tPin;
	pinMode(tPin, OUTPUT);														// setting the pin to output mode
}
void LD::poll() {
	if ((nTime == 0) || (nTime > millis())) return;								// nothing to do or wrong time to do

	if (mode == 0) {															// led off
		off();
		nTime = 0;

	} else if (mode == 1) {														// led on
		on();
		nTime = 0;

	} else if (mode == 2) {														// blink slow
		toggle();
		nTime = millis() + slowRate;
		
	} else if (mode == 3) {														// blink fast
		toggle();
		nTime = millis() + fastRate;
	
	} else if (mode == 4) {														// blink short one 
		if (!bCnt++) {
			on();
			nTime = millis() + fastRate;
		} else set(0);
		
	} else if (mode == 5) {														// blink short three
		if (bCnt++ >= 5) set(0);
		toggle();
		nTime = millis() + fastRate;
		
	} else if (mode == 6) {														// heartbeat
		if (bCnt > 3) bCnt = 0;
		toggle();
		nTime = millis() + heartBeat[bCnt++];
	}

}
void LD::set(uint8_t tMode) {
	mode = tMode;
	bCnt = 0;
	nTime = millis();
}
void LD::stop() {
	digitalWrite(pin,0);														// switch led off
	mode = 0;
	bCnt = 0;
	nTime = 0;
	state = 0;
}
void LD::shortBlink() {
	on();
	delay(50);
	off();
	delay(50);
}
void LD::shortBlink3() {
	shortBlink();
	shortBlink();
	shortBlink();	
}

void LD::on() {
	digitalWrite(pin,1);														// switch led on
	state = 1;
}
void LD::off() {
	digitalWrite(pin,0);														// switch led off
	state = 0;
}
void LD::toggle() {
	if (state) off();
	else on();
}


//- -----------------------------------------------------------------------------------------------------------------------
//- AskSin protocol functions ---------------------------------------------------------------------------------------------
//- with a lot of support from martin876 at FHEM forum
//- -----------------------------------------------------------------------------------------------------------------------
// general functions for initializing and operating of module
HM::HM(s_jumptable *jtPtr, void *mcPtr) {
	jTblPtr = jtPtr;															// jump table for call back functions
	mcConfPtr = (uint16_t)mcPtr;												// store pointer to main channel structure
}
void HM::init() {																// starts also the send/receive class for the rf module
	cc.init();																	// init the TRX module
	initRegisters();															// init the storage management module
	setPowerMode(0);															// set default power mode of HM device
	delay(100);																	// otherwise we get a problem with serial console
	enableIRQ_GDO0();															// attach callback function for GDO0 (INT0)
}
void HM::poll() {																// task scheduler
	if (recv.data[0] > 0) recv_poll();											// trace the received string and decide what to do further
	if (send.counter > 0) send_poll();											// something to send in the buffer?
	if (conf.act > 0) send_conf_poll();											// some config to be send out
	if (pevt.act > 0) send_peer_poll();											// send peer events
	power_poll();
	ld.poll();
}
void HM::send_out() {
	if (bitRead(send.data[2],5)) send.retries = maxRetries;						// check for ACK request and set max retries counter
	else send.retries = 1;														// otherwise send only one time

	send.burst = bitRead(send.data[2],4);										// burst necessary? 

	if (memcmp(&send.data[7], HMID, 3) == 0) {									// if the message is addressed to us, 
		memcpy(recv.data,send.data,send.data[0]+1);								// then copy in receive buffer. could be the case while sending from serial console
        }
        send.counter = 1;
        
        /*
		send.counter = 0;														// no need to fire
	} else {																	// it's not for us, so encode and put in send queue

		send.counter = 1;														// and fire
	}*/
}
void HM::reset(void) {
	setEEpromBlock((uint16_t)&ee->magNbr,2,(uint8_t*) &broadCast);							// clear magic byte in eeprom and step in initRegisters
	initRegisters();															// reload the registers
	ld.stop();																	// stop blinking
	ld.shortBlink3();															// blink three times short
}
void HM::setConfigEvent(void) {
	s_jumptable x;
	for (s_jumptable* p = jTblPtr; ; ++p) {										// find the call back function
		x.code = pgm_read_byte(&p->code);										// get back variables, because they are in program memory
		x.spec = pgm_read_byte(&p->spec);
		x.fun =  (void (*)(uint8_t, uint8_t*, uint8_t))pgm_read_word(&p->fun);
		
		if ((x.code == 0xFF) && (x.spec == 0xFF)) {
			x.fun(0,(uint8_t*) &broadCast,0);
			break;																// and jump into
		}
	}
}

void HM::setPowerMode(uint8_t mode) {
	// there are 3 power modes for the TRX868 module
	// TX mode will switched on while something is in the send queue
	// 0 - RX mode enabled by default, take approx 17ma
	// 1 - RX is in burst mode, RX will be switched on every 250ms to check if there is a carrier signal
	//     if yes - RX will stay enabled until timeout is reached, prolongation of timeout via receive function seems not necessary
	//				to be able to receive an ACK, RX mode should be switched on by send function
	//     if no  - RX will go in idle mode and wait for the next carrier sense check
	// 2 - RX is off by default, TX mode is enabled while sending something
	//     configuration mode is required in this setup to be able to receive at least pairing and config request strings
	//     should be realized by a 30 sec timeout function for RX mode
	// as output we need a status indication if TRX868 module is in receive, send or idle mode
	// idle mode is then the indication for power down mode of AVR

	switch (mode) {
		case 1:																	// no power savings, RX is in receiving mode
			powr.mode = 1;														// set power mode
			set_sleep_mode(SLEEP_MODE_IDLE);									// normal power saving
			break;

		case 2:																	// some power savings, RX is in burst mode
			powr.mode = 2;														// set power mode
			powr.parTO = 15000;													// pairing timeout
			powr.minTO = 2000;													// stay awake for 2 seconds after sending
			powr.nxtTO = millis() + 250;										// check in 250ms for a burst signal

			MCUSR &= ~(1<<WDRF);												// clear the reset flag
			WDTCSR |= (1<<WDCE) | (1<<WDE);										// set control register to change enabled and enable the watch dog
			WDTCSR = 1<<WDP2;													// 250 ms
			powr.wdTme = 256;													// store the watch dog time for adding in the poll function 
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);								// max power saving
			break;

		case 3:																	// most power savings, RX is off beside a special function where RX stay in receive for 30 sec
			MCUSR &= ~(1<<WDRF);												// clear the reset flag
			WDTCSR |= (1<<WDCE) | (1<<WDE);										// set control register to change enabled and enable the watch dog
			//WDTCSR = 1<<WDP2;													// 250 ms
			//WDTCSR = 1<<WDP1 | 1<<WDP2;										// 1000 ms
			//WDTCSR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2;								// 2000 ms
			WDTCSR = 1<<WDP0 | 1<<WDP3;											// 8000 ms
			powr.wdTme = 8190;													// store the watch dog time for adding in the poll function

		case 4:																	// most power savings, RX is off beside a special function where RX stay in receive for 30 sec
			powr.mode = mode;													// set power mode
			powr.parTO = 15000;													// pairing timeout
			powr.minTO = 1000;													// stay awake for 1 seconds after sending
			powr.nxtTO = millis() + 4000;										// stay 4 seconds awake to finish boot time
			
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);								// max power saving

			ld.set(2);															// blink the led to show it is awake
			break;
		default:																// no power saving, same as case 0, if user had chosen a wrong power saving mode
			powr.mode = 0;														// set power mode
	}
	powr.state = 1;																// after init of the TRX module it is in RX mode
	//Serial << "pwr.mode:" << powr.mode << '\n';
}
void HM::stayAwake(uint32_t xMillis) {
	if (powr.state == 0) cc.detectBurst();										// if TRX is in sleep, switch it on	
	powr.state = 1;																// remember TRX state
	powr.nxtTO = millis() + xMillis;											// stay awake for some time by setting next check time

}

// external functions for pairing and communicating with the module
void HM::startPairing(void) {													// send a pairing request to master
	//                               01 02    03                            04 05 06 07
	// 1A 00 A2 00 3F A6 5C 00 00 00 10 80 02 50 53 30 30 30 30 30 30 30 31 9F 04 01 01
	if (powr.mode > 1) stayAwake(powr.parTO);									// stay awake for the next 30 seconds
	memcpy_P(send_payLoad, devParam, 17);										// copy details out of register.h
	send_prep(send.mCnt++,0xA2,0x00,regDev.pairCentral,send_payLoad,17);
}
void HM::sendInfoActuatorStatus(uint8_t cnl, uint8_t status, uint8_t flag) {
	if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;					// not paired, nothing to send

//	"10;p01=06"   => { txt => "INFO_ACTUATOR_STATUS", params => {
//		CHANNEL => "2,2",
//		STATUS  => '4,2',
//		UNKNOWN => "6,2",
//		RSSI    => '08,02,$val=(-1)*(hex($val))' } },
	send_payLoad[0] = 0x06;														// INFO_ACTUATOR_STATUS
	send_payLoad[1] = cnl;														// channel
	send_payLoad[2] = status;													// status
	send_payLoad[3] = flag;														// unknown
	send_payLoad[4] = cc.trx868.rssi;											// RSSI
	
	// if it is an answer to a CONFIG_STATUS_REQUEST we have to use the same message id as the request 
	uint8_t tCnt;
	if ((recv.data[3] == 0x01) && (recv.data[11] == 0x0E)) tCnt = recv_rCnt;
	else tCnt = send.mCnt++;
	send_prep(tCnt,0xA4,0x10,regDev.pairCentral,send_payLoad,5);				// prepare the message
}
void HM::sendACKStatus(uint8_t cnl, uint8_t status, uint8_t douolo) {
	//if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;					// not paired, nothing to send

//	"02;p01=01"   => { txt => "ACK_STATUS",  params => {
//		CHANNEL        => "02,2",
//		STATUS         => "04,2",
//		DOWN           => '06,02,$val=(hex($val)&0x20)?1:0',
//		UP             => '06,02,$val=(hex($val)&0x10)?1:0',
//		LOWBAT         => '06,02,$val=(hex($val)&0x80)?1:0',
//		RSSI           => '08,02,$val=(-1)*(hex($val))', }},
	send_payLoad[0] = 0x01;														// ACK Status
	send_payLoad[1] = cnl;														// channel
	send_payLoad[2] = status;													// status
	send_payLoad[3] = douolo;													// down, up, low battery
	send_payLoad[4] = cc.trx868.rssi;											// RSSI
	
	// l> 0E EA 80 02 1F B7 4A 63 19 63 01 01 C8 00 4B
	//send_prep(recv_rCnt,0x80,0x02,regDev.pairCentral,send_payLoad,5);	// prepare the message
	send_prep(recv_rCnt,0x80,0x02,recv_reID,send_payLoad,5);					// prepare the message
}
void HM::sendSensorData(uint32_t energyCounter, uint32_t power, uint16_t current, uint16_t voltage, uint8_t frequency) {
	if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;					// not paired, nothing to send

        // energy counter 3 Bytes
        // power: 3 Bytes
        // current: 2 Bytes
        // voltage: 2 Bytes
        // frequency: 1 Byte
        
	send_payLoad[0] = energyCounter >> 16;
	send_payLoad[1] = energyCounter >> 8;
	send_payLoad[2] = energyCounter;
	send_payLoad[3] = power >> 16;
	send_payLoad[4] = power >> 8;
	send_payLoad[5] = power;
        send_payLoad[6] = current >> 8;
        send_payLoad[7] = current;
        send_payLoad[8] = voltage >> 8;
        send_payLoad[9] = voltage;
        send_payLoad[10] = frequency;

	send_prep(send.mCnt++,0x80,0x5E,regDev.pairCentral,send_payLoad,11);				// prepare the message															// short led blink
}
void HM::sendPeerREMOTE(uint8_t button, uint8_t longPress, uint8_t lowBat) {
	// no data needed, because it is a (40)REMOTE EVENT
	// "40"          => { txt => "REMOTE"      , params => {
	//	   BUTTON   => '00,2,$val=(hex($val)&0x3F)',
	//	   LONG     => '00,2,$val=(hex($val)&0x40)?1:0',
	//	   LOWBAT   => '00,2,$val=(hex($val)&0x80)?1:0',
	//	   COUNTER  => "02,2", } },
	if (button > maxChannel) return;											// channel out of range, do nothing
	if (pevt.act) {																// already sending an event
		if (longPress == 2) pevt.eol = 1;										// someone would send an long key release message
		return;	
	}
	
	if (doesListExist(button,4) == 0) {											// check if a list4 exist, otherwise leave
		//Serial << "sendPeerREMOTE failed\n";
		return;
	}

	// set variables in struct and make send_peer_poll active
	pevt.cnl = button;															// peer database channel
	pevt.type = 0x40;															// message type
	pevt.mFlg = (uint8_t)((longPress == 1)?0x84:0xA4);							// no ACK needed while long key press is send
	pevt.data[0] = button | ((longPress)?1:0) << 6 | lowBat << 7;				// construct message
	pevt.data[1] = pevt.mCnt[pevt.cnl-1]++;										// increase event counter, important for switch event
	pevt.len = 2;																// 2 bytes payload
	
	pevt.act = 1;																// active, 1 = yes, 0 = no
	ld.shortBlink();															// short led blink
}
void HM::sendPeerRAW(uint8_t cnl, uint8_t type, uint8_t *data, uint8_t len) {
	// validate the input, and fill the respective variables in the struct
	// message handling is taken from send_peer_poll
	if (cnl > maxChannel) return;												// channel out of range, do nothing
	if (pevt.act) return;														// already sending an event, leave
	if (doesListExist(cnl,4) == 0) {											// check if a list4 exist, otherwise leave
		//Serial << "sendPeerREMOTE failed\n";
		return;
	}
	
	// set variables in struct and make send_peer_poll active
	pevt.cnl = cnl;																// peer database channel
	pevt.type = type;															// message type
	pevt.mFlg = 0xA2;
	
	if (len > 0) {																// copy data if there are some
		memcpy(pevt.data, data, len);											// data to send
		pevt.len = len;															// len of data to send
	}
	pevt.act = 1;																// active, 1 = yes, 0 = no
	ld.shortBlink();															// short led blink
}
void HM::send_ACK(void) {
	uint8_t payLoad[] = {0x00};	 												// ACK
	send_prep(recv_rCnt,0x80,0x02,recv_reID,payLoad,1);
}
void HM::send_NACK(void) {
	uint8_t payLoad[] = {0x80};													// NACK
	send_prep(recv_rCnt,0x80,0x02,recv_reID,payLoad,1);
}
#if defined(USE_SERIAL)
// some debug functions
void HM::printSettings() {
	Serial << F("Serial: ");
	for (int i = 0; i < 10; i++) {												// serial number has 10 bytes
		Serial << (char)pgm_read_byte(&(devParam[i+3]));						// get the serial number from program space
	}

	Serial << F(", Model ID: ");
	for (int i = 0; i < 2; i++) {												// model id has 2 bytes
		Serial << pHex(pgm_read_byte(&(devParam[i+1])))	<< ' ';					// get the model id from program space
	}

	Serial << F(", HMID: ") << pHex(HMID,3) << '\n';							// displays the own id
	Serial << F("Paired: ") << pHex(regDev.pairCentral,3) << F("\n\n");			// pairing id
}
void HM::printConfig() {
	s_slcVar sV;																// size some variables
	uint8_t peer[] = {0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00};
	
	// show device config
	Serial << F("\nDevice config, size: ") << sizeof(ee) << F(" byte\n");
	Serial << F("cnl\tlst\tslcPtr\tslcEnd\tphyAddr\tphyLen\n");
	
	for (uint8_t i = 0; i < 20; i++) {											// count through the channel
		for (uint8_t j = 0; j < 20; j++) {										// count through the lists
			if (!getSliceDetail(i, j, &peer[0], &sV)) continue;					// get the slice details
			Serial << i << '\t' << j << '\t' << sV.slcPtr << '\t' << sV.slcLen  << '\t' << sV.phyAddr  << '\t' << sV.phyLen << '\n';
		}
	}
	
	// step through the complete channel list
	Serial << F("\nPeer Database, size: ") << sizeof(peerdb) << F(" byte\n");	// some headline
	Serial << F("cnl\tpIdx\tslcPtr\tslcEnd\tphyAddr\tphyLen\tlist\tpeer\n");
	
	for (uint8_t i = 0; i < 20; i++) {											// step through all channels
		// check if we have a list3 or list4
		uint8_t lst = 0;
		if (doesListExist(i+1, 3)) lst = 3;										// check for list3
		if (doesListExist(i+1, 4)) lst = 4;										// check for list4
		if (lst == 0) continue;													// no list3 or list4 therefore next i
		
		// step through the peers
		for (uint8_t j = 0; j < 20; j++) {										// step through all possible peers
			if (!getPeerByIdx(i+1,j,&peer[4])) continue;						// ask for the peer by channel and peer index
			peer[3] = j;														// store the peer channel byte
			if (!getSliceDetail(i+1, lst, &peer[0], &sV)) continue;				// ask for the slice details as list3
			Serial << (i+1) << '\t' << (j) << '\t' << sV.slcPtr << '\t' << sV.slcLen  << '\t' << sV.phyAddr  << '\t' << sV.phyLen  << '\t' << lst << '\t';	// print the slice details
			Serial << pHex(&peer[4],4) << '\n';									// and print the peer incl peer channel
		}
	}
	Serial << '\n';
}
#endif
	
//- private: //------------------------------------------------------------------------------------------------------------
// hardware definition for interrupt handling
void HM::isrGDO0event(void) {
	disableIRQ_GDO0();															// disable interrupt otherwise we could get some new content while we copy the buffer

	if (hm.cc.receiveData(hm.recv.data)) {										// is something in the receive string
		hm.hm_dec(hm.recv.data);												// decode the content
	}

	enableIRQ_GDO0();															// enable the interrupt again
}

// some polling functions
void HM::recv_poll(void) {														// handles the receive objects

	// do some checkups 
	if (memcmp(&recv.data[7], HMID, 3) == 0) recv.forUs = 1;					// for us 
	else recv.forUs = 0;
	if (memcmp(&recv.data[7], broadCast, 3) == 0) recv.bCast = 1;						// or a broadcast 
	else recv.bCast = 0;														// otherwise only a log message
	
	// show debug message
	#if defined(AS_DBG)															// some debug message
	if(recv.forUs) Serial << F("-> ");
	else if(recv.bCast) Serial << F("b> ");
	else Serial << F("l> ");
	Serial << pHexL(recv.data, recv.data[0]+1) << pTime();
	exMsg(recv.data);															// explain message
	#endif

	// is the message from a valid sender (pair or peer), if not then exit - takes ~2ms
	if ((isPairKnown(recv_reID) == 0) && (isPeerKnown(recv_reID) == 0)) {		// check against peers
		#if defined(AS_DBG)													// some debug message
		//Serial << "pair/peer did not fit, exit\n";
		#endif
		recv.data[0] = 0;														// clear receive string
		return;
	}

	// check if it was a repeated message, delete while already received - takes ~2ms
	if (bitRead(recv.data[2],6)) {												// check repeated flag
		bitSet(recv.p_data[2],6);												// set repeated flag in prev received string
		uint16_t ret = memcmp(recv.p_data,recv.data,recv.data[0]+1);			// compare with already received string
		
		#if defined(AS_DBG)														// some debug message
		Serial << F("   repeated message; ");
		if (ret == 0) Serial << F("already received - skip\n");
		else Serial << F("not received before...\n");
		#endif
		
		if (ret == 0) {															// already received
			recv.data[0] = 0;													// therefore ignore
			return;																// and skip
		}

	}
	memcpy(recv.p_data,recv.data,recv.data[0]+1);								// save received string for next compare

	// decide where to jump in
	if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x01)) {					// message is a config message
		if      (recv_by11 == 0x01) recv_ConfigPeerAdd();						// 01, 01
		else if (recv_by11 == 0x02) recv_ConfigPeerRemove();					// 01, 02
		else if (recv_by11 == 0x03) recv_ConfigPeerListReq();					// 01, 03
		else if (recv_by11 == 0x04) recv_ConfigParamReq();						// 01, 04
		else if (recv_by11 == 0x05) recv_ConfigStart();							// 01, 05
		else if (recv_by11 == 0x06) recv_ConfigEnd();							// 01, 06
		else if (recv_by11 == 0x08) recv_ConfigWriteIndex();					// 01, 08
		else if (recv_by11 == 0x09) recv_ConfigSerialReq();						// 01, 09
		else if (recv_by11 == 0x0A) recv_Pair_Serial();							// 01, 0A
		else if (recv_by11 == 0x0E) recv_ConfigStatusReq();						// 01, 0E
		#if defined(AS_DBG)														// some debug message
		else Serial << F("\nUNKNOWN MESSAGE, PLEASE REPORT!\n\n");
		#endif
	}
	
	// l> 0A 73 80 02 63 19 63 2F B7 4A 00
	if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x02)) {					// message seems to be an ACK
		send.counter = 0;
	}
	
	if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x11)) {
		recv_PairEvent();
	}

	if((recv.forUs) && (recv_isMsg) && (recv_msgTp >= 0x12)) {
		recv_PeerEvent();
	}
	
	//to do: if it is a broadcast message, do something with
	recv.data[0] = 0;															// otherwise ignore
}
void HM::send_poll(void) {														// handles the send queue
	if((send.counter <= send.retries) && (send.timer <= millis())) {			// not all sends done and timing is OK
		
		// here we encode and send the string
		hm_enc(send.data);														// encode the string
		disableIRQ_GDO0();														// disable interrupt otherwise we could get some new content while we copy the buffer

		cc.sendData(send.data,send.burst);										// and send
		enableIRQ_GDO0();														// enable the interrupt again
		hm_dec(send.data);														// decode the string
		
		// setting some variables
		send.counter++;															// increase send counter
		send.timer = millis() + timeOut;										// set the timer for next action
		powr.state = 1;															// remember TRX module status, after sending it is always in RX mode
		if ((powr.mode > 0) && (powr.nxtTO < (millis() + powr.minTO))) stayAwake(powr.minTO); // stay awake for some time

		#if defined(AS_DBG)														// some debug messages
		Serial << F("<- ") << pHexL(send.data, send.data[0]+1) << pTime();
		#endif

	}
			
	if((send.counter > send.retries) && (send.counter < maxRetries)) {			// all send but don't wait for an ACK
		send.counter = 0; send.timer = 0;										// clear send flag
	}
			
	if((send.counter > send.retries) && (send.timer <= millis())) {				// max retries achieved, but seems to have no answer
		send.counter = 0; send.timer = 0;										// cleanup of send buffer
		// todo: error handling, here we could jump some were to blink a led or whatever
		
		#if defined(AS_DBG) 
		Serial << F("-> NA ") << pTime();
		#endif
	}
}																															// ready, should work
void HM::send_conf_poll(void) {
	if (send.counter > 0) return;												// send queue is busy, let's wait
	uint8_t len;

	if (conf.type == 0x01) {
		// answer                            Cnl  Peer         Peer         Peer         Peer
		// l> 1A 05 A0 10 1E 7A AD 63 19 63  01   1F A6 5C 02  1F A6 5C 01  11 22 33 02  11 22 33 01
		//                                   Cnl  Termination
		// l> 0E 06 A0 10 1E 7A AD 63 19 63  01   00 00 00 00
		len = getPeerListForMsg(conf.channel, send_payLoad+1);					// get peer list
		if (len == 0x00) {														// check if all done
			memset(&conf, 0, sizeof(conf));										// clear the channel struct
			return;																// exit
		} else if (len == 0xff) {												// failure, out of range
			memset(&conf, 0, sizeof(conf));										// clear the channel struct
			send_NACK();
		} else {																// seems to be ok, answer
			send_payLoad[0] = 0x01;												// INFO_PEER_LIST
			send_prep(conf.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);		// prepare the message
			//send_prep(send.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);		// prepare the message
		}
	} else if (conf.type == 0x02) {
		// INFO_PARAM_RESPONSE_PAIRS message
		//                               RegL_01:  30:06 32:50 34:4B 35:50 56:00 57:24 58:01 59:01 00:00
		// l> 1A 04 A0 10  1E 7A AD  63 19 63  02  30 06 32 50 34 4B 35 50 56 00 57 24 58 01 59 01 (l:27)(131405)
	
		//Serial << "hab dich\n";
		len = getListForMsg2(conf.channel, conf.list, conf.peer, send_payLoad+1); // get the message
		if (len == 0) {															// check if all done
			memset(&conf, 0, sizeof(conf));										// clear the channel struct
			return;																// and exit
		} else if (len == 0xff) {												// failure, out of range
			memset(&conf, 0, sizeof(conf));										// clear the channel struct
			send_NACK();
		} else {																// seems to be ok, answer
			send_payLoad[0] = 0x02;			 									// INFO_PARAM_RESPONSE_PAIRS
			send_prep(conf.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);		// prepare the message
			//send_prep(send.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);		// prepare the message
		}
	} else if (conf.type == 0x03) {
		// INFO_PARAM_RESPONSE_SEQ message
		//                               RegL_01:  30:06 32:50 34:4B 35:50 56:00 57:24 58:01 59:01 00:00
		// l> 1A 04 A0 10  1E 7A AD  63 19 63  02  30 06 32 50 34 4B 35 50 56 00 57 24 58 01 59 01 (l:27)(131405)
	
	}
	
}
void HM::send_peer_poll(void) {
	// go through the peer database and get the idx per slot, load the respective list4
	// if no peer exist in the respective channel then send to master
	// send out the message accordingly, loop until send_poll is clear and start the next peer.
	// if the message was a long key press, then prepare the struct for sending out a last message 
	// with ACK requested

	if (send.counter > 0) return;												// something is in the send queue, lets wait for the next free slot

	// we are in a loop, therefore check if the request is completed and clear struct
	if (pevt.idx >= peermax[pevt.cnl-1]) {										// we are through the list of peers, clear variables
		// check if a message was send to at least on device, if not send to master
		if (pevt.sta == 0) send_prep(send.mCnt++,(bitRead(pevt.mFlg,5)?0xA2:0x82),pevt.type,regDev.pairCentral,pevt.data,pevt.len);
		
		if ((!bitRead(pevt.mFlg,5)) && (pevt.eol)) {							// seems to be the end of a long key press series
			pevt.idx = 0;														// start again from 0
			pevt.sta = 0;														// clear status message flag
			bitSet(pevt.mFlg,5);												// set ACK flag
			//Serial << "hab dich\n";
		} else {
			pevt.idx = 0; pevt.sta = 0;	pevt.eol = 0; pevt.act = 0;				// clear struct object, no need to jump in again
			return;
		}
		
	}
	
	// prepare the next peer address
	//Serial << "cnl: " << pevt.cnl << ", idx: " << pevt.idx << '\n';
	uint8_t peerBuf[4];
	uint8_t ret = getPeerByIdx(pevt.cnl,pevt.idx,peerBuf);						// get the respective peer from database
	if ((memcmp(peerBuf,broadCast,4) == 0) || (ret == 0)) {						// if peer is empty, increase the idx and leave while we are in a loop
		pevt.idx++;
		return;
	}
	
	// get the respective list4
	s_slcVar sV;																// some declarations
	uint8_t regLstByte = 0;

	ret = getSliceDetail(pevt.cnl, 4, peerBuf, &sV);							// get cnl list4 
	if (ret) {
		// at the moment we are looking only for register address 1 in list 4 (peerNeedsBurst, expectAES), list 4 will not be available in user space
		
		uint8_t tLst[sV.phyLen];												// size a variable	
		ret = getRegList(sV.slcPtr, sV.slcLen, tLst);							// get the register in the variable
		
		void *x = memchr(tLst, 0x01, sV.phyLen);								// search the character in the address string
		if ((uint16_t)x) {														// if we found the searched string
			uint16_t dataPtr = (uint16_t)x-(uint16_t)tLst;						// calculate the respective address in list
			regLstByte = getEEpromByte(dataPtr+(uint16_t)&ee->regs+sV.phyAddr);	// get the respective byte
			//Serial << "get byte: " << pHex(regLstByte) << '\n';
			//Serial << "dataPtr:" << dataPtr << ", phyAddr:" << sV.phyAddr << ", eepromAddr:" << (uint16_t)&sEEPROM->regs << '\n';
		}
		
		//Serial << "peer: " << pHex(peerBuf,4) << ", tLst: " << pHex(tLst,sV.phyLen) << ", rB: " << regLstByte << '\n';
	}
	
	// in regLstByte there are two information. peer needs AES and burst needed
	// AES will be ignored at the moment, but burst needed will be translated into the message flag - bit 0 in regLstByte, translated to bit 4 = burst transmission in msgFlag
	uint8_t mFlg = pevt.mFlg;													// copy the message flag
	mFlg |= bitRead(regLstByte,0) << 4;											// read the need burst flag
	
	// prepare send string and increase timer
	send_prep(send.mCnt++,mFlg,pevt.type,peerBuf,pevt.data,pevt.len);			// prepare the message
	pevt.sta = 1;																// indicates that we had found a peer and string was send
	pevt.idx++;																	// increase idx for next try
}
void HM::power_poll(void) {
	// there are 3 power modes for the TRX868 module
	// TX mode will switched on while something is in the send queue
	// 1 - RX mode enabled by default, take approx 17ma
	// 2 - RX is in burst mode, RX will be switched on every 250ms to check if there is a carrier signal
	//     if yes - RX will stay enabled until timeout is reached, prolongation of timeout via receive function seems not necessary
	//				to be able to receive an ACK, RX mode should be switched on by send function
	//     if no  - RX will go in idle mode and wait for the next carrier sense check
	// 3 - RX is off by default, TX mode is enabled while sending something
	//     configuration mode is required in this setup to be able to receive at least pairing and config request strings
	//     should be realized by a 15 sec timeout function for RX mode
	//     system time in millis will be hold by a regular wakeup from the watchdog timer
	// 4 - Same as power mode 3 but without watchdog
	
	if (powr.mode == 0) return;													// in mode 0 there is nothing to do
	if (powr.nxtTO > millis()) return;											// no need to do anything
	if (send.counter > 0) return;												// send queue not empty
	
	// power mode 2, module is in sleep and next check is reached
	if ((powr.mode == 2) && (powr.state == 0)) {
		if (cc.detectBurst()) {													// check for a burst signal, if we have one, we should stay awake
			powr.nxtTO = millis() + powr.minTO;									// schedule next timeout with some delay
		} else {																// no burst was detected, go to sleep in next cycle
			powr.nxtTO = millis();												// set timer accordingly	
		}
		powr.state = 1;															// set status to awake
		return;
	}

	// power mode 2, module is active and next check is reached
	if ((powr.mode == 2) && (powr.state == 1)) {
		cc.setPowerDownxtStatte();													// go to sleep
		powr.state = 0;
		powr.nxtTO = millis() + 250;											// schedule next check in 250 ms
	}

	// 	power mode 3, check RX mode against timer. typically RX is off beside a special command to switch RX on for at least 30 seconds
	if ((powr.mode >= 3) && (powr.state == 1)) {
		cc.setPowerDownxtStatte();													// go to sleep
		powr.state = 0;
	}

	// sleep for mode 2, 3 and 4
	if ((powr.mode > 1) && (powr.state == 0)) {									// TRX module is off, so lets sleep for a while
		ld.stop();																// stop blinking, because we are going to sleep
		if ((powr.mode == 2) || (powr.mode == 3)) WDTCSR |= (1<<WDIE);			// enable watch dog if power mode 2 or 3

		ADCSRA = 0;																// disable ADC
		uint8_t xPrr = PRR;														// turn off various modules
		PRR = 0xFF;

		sleep_enable();															// enable the sleep mode
		
//		MCUCR |= (1<<BODS) | (1<<BODSE);										// turn off brown-out enable in software
//		MCUCR &= ~(1<<BODSE);													// must be done right before sleep
		sleep_cpu();															// goto sleep

		/* wake up here */
		
		sleep_disable();														// disable sleep
		if ((powr.mode == 2) || (powr.mode == 3)) WDTCSR &= ~(1<<WDIE);													// disable watch dog
		PRR = xPrr;																// restore modules
		
		if (wd_flag == 1) {														// add the watchdog time to millis()
			wd_flag = 0;														// to detect the next watch dog timeout
			timer0_millis += powr.wdTme;										// add watchdog time to millis() function
		} else {
			stayAwake(powr.minTO);												// stay awake for some time, if the wakeup where not raised from watchdog
		}
		ld.set(2);																// blink the led to show it is awake
	}
}

// receive message handling
void HM::recv_ConfigPeerAdd(void) {	
	// description --------------------------------------------------------
	//                                  Cnl      PeerID    PeerCnl_A  PeerCnl_B
	// l> 10 55 A0 01 63 19 63 1E 7A AD 03   01  1F A6 5C  06         05

	// do something with the information ----------------------------------
	addPeerFromMsg(recv_payLoad[0], recv_payLoad+2);

	// send appropriate answer ---------------------------------------------
	// l> 0A 55 80 02 1E 7A AD 63 19 63 00
	if (recv_ackRq) send_ACK();														// send ACK if requested
	//if ((recv_ackRq) && (ret == 1)) send_ACK();
	//else if (recv_ackRq) send_NACK();
}
void HM::recv_ConfigPeerRemove(void) {
	// description --------------------------------------------------------
	//                                  Cnl      PeerID    PeerCnl_A  PeerCnl_B
	// l> 10 55 A0 01 63 19 63 1E 7A AD 03   02  1F A6 5C  06         05

	// do something with the information ----------------------------------
	removePeerFromMsg(recv_payLoad[0], recv_payLoad+2);

	// send appropriate answer ---------------------------------------------
	// l> 0A 55 80 02 1E 7A AD 63 19 63 00
	if (recv_ackRq) send_ACK();
	//if ((recv_ackRq) && (ret == 1)) send_ACK();									// send ACK if requested
	//else if (recv_ackRq) send_NACK();
}
void HM::recv_ConfigPeerListReq(void) {
	// description --------------------------------------------------------
	//                                  Cnl 
	// l> 0B 05 A0 01 63 19 63 1E 7A AD 01  03

	// do something with the information ----------------------------------
	conf.mCnt = recv_rCnt;
	conf.channel = recv_payLoad[0];
	memcpy(conf.reID, recv_reID, 3);
	conf.type = 0x01;

	// send appropriate answer ---------------------------------------------
	// answer will be generated in config_poll function
	conf.act = 1;
}
void HM::recv_ConfigParamReq(void) {
	// description --------------------------------------------------------
	//                                  Cnl    PeerID    PeerCnl  ParmLst
	// l> 10 04 A0 01 63 19 63 1E 7A AD 01  04 00 00 00  00       01
	// do something with the information ----------------------------------
	conf.mCnt = recv_rCnt;
	conf.channel = recv_payLoad[0];
	conf.list = recv_payLoad[6];
	memcpy(conf.peer, &recv_payLoad[2], 4);
	memcpy(conf.reID, recv_reID, 3);
	// todo: check when message type 2 or 3 is selected
	conf.type = 0x02;

	// send appropriate answer ---------------------------------------------
	// answer will be generated in config_poll function
	conf.act = 1;
}
void HM::recv_ConfigStart(void) {
	// description --------------------------------------------------------
	//                                  Cnl    PeerID    PeerCnl  ParmLst
	// l> 10 01 A0 01 63 19 63 1E 7A AD 00  05 00 00 00  00       00

	// do something with the information ----------------------------------
	// todo: check against known master id, if master id is empty, set from everyone is allowed
	conf.channel = recv_payLoad[0];												// set parameter
	memcpy(conf.peer,&recv_payLoad[2],4);
	conf.list = recv_payLoad[6];
	conf.wrEn = 1;																// and enable write to config
	
	// send appropriate answer ---------------------------------------------
	if (recv_ackRq) send_ACK();													// send ACK if requested
}
void HM::recv_ConfigEnd(void) {
	// description --------------------------------------------------------
	//                                  Cnl
	// l> 0B 01 A0 01 63 19 63 1E 7A AD 00  06

	// do something with the information ----------------------------------
	conf.wrEn = 0;																// disable write to config
	getMainChConfig();															// probably something changed, reload config
	setConfigEvent();															// raise a config had changed event in user space
	
	// send appropriate answer ---------------------------------------------
	if (recv_ackRq) send_ACK();													// send ACK if requested
}
void HM::recv_ConfigWriteIndex(void) {
	// description --------------------------------------------------------
	//                                  Cnl    Data
	// l> 13 02 A0 01 63 19 63 1E 7A AD 00  08 02 01 0A 63 0B 19 0C 63

	// do something with the information ----------------------------------
	if ((!conf.wrEn) || (!(conf.channel == recv_payLoad[0]))) {					// but only if we are in config mode
		#if defined(AS_DBG)
		Serial << F("   write data, but not in config mode\n");
		#endif
		return;
	}
	uint8_t payLen = recv_len - 11;												// calculate len of payload and provide the data
	uint8_t ret = setListFromMsg(conf.channel, conf.list, conf.peer, &recv_payLoad[2], payLen);
	//Serial << "we: " << conf.wrEn << ", cnl: " << conf.channel << ", lst: " << conf.list << ", peer: " << pHex(conf.peer,4) << '\n';
	//Serial << "pl: " << pHex(&recv_payLoad[2],payLen) << ", ret: " << ret << '\n';

	// send appropriate answer ---------------------------------------------
	if ((recv_ackRq) && (ret == 1))send_ACK();									// send ACK if requested
	else if (recv_ackRq) send_NACK();											// send NACK while something went wrong
}
void HM::recv_ConfigSerialReq(void) {
	// description --------------------------------------------------------
	// l> 0B 48 A0 01 63 19 63 1E 7A AD 00 09

	// do something with the information ----------------------------------
	// nothing to do, we have only to answer

	// send appropriate answer ---------------------------------------------
	//                                     SerNr
	// l> 14 48 80 10 1E 7A AD 63 19 63 00 4A 45 51 30 37 33 31 39 30 35
	send_payLoad[0] = 0x00; 													// INFO_SERIAL
	memcpy_P(&send_payLoad[1], &devParam[3], 11);								// copy details out of register.h
	send_prep(recv_rCnt,0x80,0x10,recv_reID,send_payLoad,11);					// prepare the message
	//send_prep(send.mCnt++,0x80,0x10,recv_reID,send_payLoad,11);				// prepare the message
}
void HM::recv_Pair_Serial(void) {
	// description --------------------------------------------------------
	//                                         Serial
	// l> 15 48 A0 01 63 19 63 1E 7A AD 00 0A  4A 45 51 30 37 33 31 39 30 35
	
	// do something with the information ----------------------------------
	// compare serial number with own serial number and send pairing string back
	if (memcmp_P(&recv_payLoad[2],&devParam[3],10) != 0) return;

	// send appropriate answer ---------------------------------------------
	// l> 1A 01 A2 00 3F A6 5C 00 00 00 10 80 02 50 53 30 30 30 30 30 30 30 31 9F 04 01 01
	memcpy_P(send_payLoad, devParam, 17);										// copy details out of register.h
	send_prep(send.mCnt++,0xA2,0x00,recv_reID,send_payLoad,17);
}
void HM::recv_ConfigStatusReq(void) {
	// description --------------------------------------------------------
	//                                   Cnl
	// l> 0B 30 A0 01 63 19 63 2F B7 4A  01  0E 

	// do something with the information ----------------------------------
	uint8_t ret = recv_Jump(0);

	// send appropriate answer ---------------------------------------------
	// answer will be send from client function; if client function was not found in jump table, we send here an empty status
	if (!ret) hm.sendInfoActuatorStatus(recv_payLoad[0], 0xff, 0);
}
void HM::recv_PeerEvent(void) {
	// description --------------------------------------------------------
	//                 peer                cnl  payload
	// -> 0B 56 A4 40  AA BB CC  3F A6 5C  03   CA

	// do something with the information ----------------------------------
	uint8_t peer[4];															// bring it in a search able format
	memcpy(peer,recv_reID,4);
	peer[3] = recv_payLoad[0] & 0xF;											// only the low byte is the channel indicator
	
	uint8_t cnl = getCnlByPeer(peer);											// check on peerdb
	if (!cnl) return;															// if peer was not found, the function returns a 0 and we can leave

	getList3ByPeer(cnl, peer);													// load list3
	recv_Jump(cnl);																// jump in user function, we do not need to check, because answer is an ACK
	
	// send appropriate answer ---------------------------------------------
	// answer should be initiated by client function in user area
	// if (recv_ackRq) send_ACK();												// send ACK if requested
}
void HM::recv_PairEvent(void) {
	// description --------------------------------------------------------
	//                 peer                    cnl  payload
	// -> 0E E6 A0 11  63 19 63  2F B7 4A  02  01   C8 00 00
	// answer is an enhanced ACK:
	// <- 0E E7 80 02 1F B7 4A 63 19 63 01 01 C8 00 54
	
	// do something with the information ----------------------------------
	recv_Jump(0);													// jump in user function, if no answer from user function, we send a blank status answer
	
	
	// send appropriate answer ---------------------------------------------
	// answer should be initiated by client function in user area
}
uint8_t HM::recv_Jump(uint8_t tCnl) {
	s_jumptable x;

	for (s_jumptable* p = jTblPtr; ; ++p) {										// find the call back function
		// check message type, should be in the list
		// if message type 01, check against byte11, return byte10 as channel and 12 to x as payload
		// l> 0B E2 A0 01 63 19 63 2F B7 4A 01 0E
		//
		// if message type 11, check against byte10, return byte11 as channel and 12 to x as payload
		// -> 0E E3 A0 11 63 19 63 2F B7 4A 02 01 C8 00 00
		//
		// if message type >12, check against nothing, return byte10 as channel and 11 to x as payload
		// -> 0B 4E A4 40 22 66 08 2F B7 4A 01 68
		//
		//Serial << "rm:" << recv_msgTp << ", by10:" << recv_by10 << ", ps:" << p->spec << '\n';
		
		x.code = pgm_read_byte(&p->code);										// get back variables, because they are in program memory
		x.spec = pgm_read_byte(&p->spec);
		x.fun =  (void (*)(uint8_t, uint8_t*, uint8_t))pgm_read_word(&p->fun);
		
		if ((recv_msgTp == 0x01) && (recv_msgTp == x.code)) {
			x.fun(recv_payLoad[0], recv_payLoad+1, recv_len - 10);				// and jump into
			return 1;
		} else if ((recv_msgTp == 0x11) && (recv_msgTp == x.code) && (recv_by10 == x.spec)) {
			x.fun(recv_payLoad[1], recv_payLoad+2, recv_len - 11);				// and jump into
			return 1;
		} else if ((recv_msgTp >= 0x12) && (recv_msgTp == x.code)) {
			x.fun(tCnl, recv_payLoad, recv_len - 9);							// and jump into
			return 1;
		}
		if (x.code == 0) break;													// break if on end of list
	}
	return 0;
}

// internal send functions
void    HM::send_prep(uint8_t msgCnt, uint8_t comBits, uint8_t msgType, uint8_t *targetID, uint8_t *payLoad, uint8_t payLen) {
	send.data[0]  = 9 + payLen;													// message length
	send.data[1]  = msgCnt;														// message counter

	// set the message flags
	//    #RPTEN    0x80: set in every message. Meaning?
	//    #RPTED    0x40: repeated (repeater operation)
	//    #BIDI     0x20: response is expected						- should be in comBits
	//    #Burst    0x10: set if burst is required by device        - will be set in peer send string if necessary
	//    #Bit3     0x08:
	//    #CFG      0x04: Device in Config mode						- peer seems to be always in config mode, message to master only if an write mode enable string was received
	//    #WAKEMEUP 0x02: awake - hurry up to send messages			- only for a master, peer don't need
	//    #WAKEUP   0x01: send initially to keep the device awake	- should be only necessary while receiving

	send.data[2]  = comBits;													// take the communication bits
	send.data[3]  = msgType;													// message type
	memcpy(&send.data[4], HMID, 3);												// source id
	memcpy(&send.data[7], targetID, 3);											// target id

	if ((uint16_t)payLoad != (uint16_t)send_payLoad) 
		memcpy(send_payLoad, payLoad, payLen);									// payload
	
	#if defined(AS_DBG)															// some debug messages
	//Serial << F("S- ") << pHexL(send.data, send.data[0]+1) << pTime();
	exMsg(send.data);															// explain message
	#endif

	send_out();
}

// some internal helpers
void HM::hm_enc(uint8_t *buffer) {

	buffer[1] = (~buffer[1]) ^ 0x89;
	uint8_t buf2 = buffer[2];
	uint8_t prev = buffer[1];

	uint8_t i;
	for (i=2; i<buffer[0]; i++) {
		prev = (prev + 0xdc) ^ buffer[i];
		buffer[i] = prev;
	}

	buffer[i] ^= buf2;
}
void HM::hm_dec(uint8_t *buffer) {

	uint8_t prev = buffer[1];
	buffer[1] = (~buffer[1]) ^ 0x89;

	uint8_t i, t;
	for (i=2; i<buffer[0]; i++) {
		t = buffer[i];
		buffer[i] = (prev + 0xdc) ^ buffer[i];
		prev = t;
	}

	buffer[i] ^= buffer[2];
}
void HM::exMsg(uint8_t *buf) {
	#if defined(AS_DBG_Explain)

	#define b_len			buf[0]
	#define b_msgTp			buf[3]
	#define b_by10			buf[10]
	#define b_by11			buf[11]

	Serial << F("   ");															// save some byte and send 3 blanks once, instead of having it in every if
	
	if        ((b_msgTp == 0x00)) {
		Serial << F("DEVICE_INFO; fw: ") << pHex(&buf[10],1) << F(", type: ") << pHex(&buf[11],2) << F(", serial: ") << pHex(&buf[13],10) << '\n';
		Serial << F("              , class: ") << pHex(&buf[23],1) << F(", pCnlA: ") << pHex(&buf[24],1) << F(", pCnlB: ") << pHex(&buf[25],1) << F(", na: ") << pHex(&buf[26],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x01)) {
		Serial << F("CONFIG_PEER_ADD; cnl: ") << pHex(&buf[10],1) << F(", peer: ") << pHex(&buf[12],3) << F(", pCnlA: ") << pHex(&buf[15],1) << F(", pCnlB: ") << pHex(&buf[16],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x02)) {
		Serial << F("CONFIG_PEER_REMOVE; cnl: ") << pHex(&buf[10],1) << F(", peer: ") << pHex(&buf[12],3) << F(", pCnlA: ") << pHex(&buf[15],1) << F(", pCnlB: ") << pHex(&buf[16],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x03)) {
		Serial << F("CONFIG_PEER_LIST_REQ; cnl: ") << pHex(&buf[10],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x04)) {
		Serial << F("CONFIG_PARAM_REQ; cnl: ") << pHex(&buf[10],1) << F(", peer: ") << pHex(&buf[12],3) << F(", pCnl: ") << pHex(&buf[15],1) << F(", lst: ") << pHex(&buf[16],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x05)) {
		Serial << F("CONFIG_START; cnl: ") << pHex(&buf[10],1) << F(", peer: ") << pHex(&buf[12],3) << F(", pCnl: ") << pHex(&buf[15],1) << F(", lst: ") << pHex(&buf[16],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x06)) {
		Serial << F("CONFIG_END; cnl: ") << pHex(&buf[10],1);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x08)) {
		Serial << F("CONFIG_WRITE_INDEX; cnl: ") << pHex(&buf[10],1) << F(", data: ") << pHex(&buf[12],(buf[0]-11));

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x09)) {
		Serial << F("CONFIG_SERIAL_REQ");
		
	} else if ((b_msgTp == 0x01) && (b_by11 == 0x0A)) {
		Serial << F("PAIR_SERIAL, serial: ") << pHex(&buf[12],10);

	} else if ((b_msgTp == 0x01) && (b_by11 == 0x0E)) {
		Serial << F("CONFIG_STATUS_REQUEST, cnl: ") << pHex(&buf[10],1);

	} else if ((b_msgTp == 0x02) && (b_by10 == 0x00)) {
		if (b_len == 0x0A) Serial << F("ACK");
		else Serial << F("ACK; data: ") << pHex(&buf[11],b_len-10);

	} else if ((b_msgTp == 0x02) && (b_by10 == 0x01)) {
		Serial << F("ACK_STATUS; cnl: ") << pHex(&buf[11],1) << F(", status: ") << pHex(&buf[12],1) << F(", down/up/loBat: ") << pHex(&buf[13],1);
		if (b_len > 13) Serial << F(", rssi: ") << pHex(&buf[14],1);

	} else if ((b_msgTp == 0x02) && (b_by10 == 0x02)) {
		Serial << F("ACK2");
		
	} else if ((b_msgTp == 0x02) && (b_by10 == 0x04)) {
		Serial << F("ACK_PROC; para1: ") << pHex(&buf[11],2) << F(", para2: ") << pHex(&buf[13],2) << F(", para3: ") << pHex(&buf[15],2) << F(", para4: ") << pHex(&buf[17],1);

	} else if ((b_msgTp == 0x02) && (b_by10 == 0x80)) {
		Serial << F("NACK");

	} else if ((b_msgTp == 0x02) && (b_by10 == 0x84)) {
		Serial << F("NACK_TARGET_INVALID");
		
	} else if ((b_msgTp == 0x03)) {
		Serial << F("AES_REPLY; data: ") << pHex(&buf[10],b_len-9);
		
	} else if ((b_msgTp == 0x04) && (b_by10 == 0x01)) {
		Serial << F("TOpHMLAN:SEND_AES_CODE; cnl: ") << pHex(&buf[11],1);

	} else if ((b_msgTp == 0x04)) {
		Serial << F("TO_ACTOR:SEND_AES_CODE; code: ") << pHex(&buf[11],1);
		
	} else if ((b_msgTp == 0x10) && (b_by10 == 0x00)) {
		Serial << F("INFO_SERIAL; serial: ") << pHex(&buf[11],10);

	} else if ((b_msgTp == 0x10) && (b_by10 == 0x01)) {
		Serial << F("INFO_PEER_LIST; peer1: ") << pHex(&buf[11],4);
		if (b_len >= 19) Serial << F(", peer2: ") << pHex(&buf[15],4);
		if (b_len >= 23) Serial << F(", peer3: ") << pHex(&buf[19],4);
		if (b_len >= 27) Serial << F(", peer4: ") << pHex(&buf[23],4);

	} else if ((b_msgTp == 0x10) && (b_by10 == 0x02)) {
		Serial << F("INFO_PARAM_RESPONSE_PAIRS; data: ") << pHex(&buf[11],b_len-10);

	} else if ((b_msgTp == 0x10) && (b_by10 == 0x03)) {
		Serial << F("INFO_PARAM_RESPONSE_SEQ; offset: ") << pHex(&buf[11],1) << F(", data: ") << pHex(&buf[12],b_len-11);

	} else if ((b_msgTp == 0x10) && (b_by10 == 0x04)) {
		Serial << F("INFO_PARAMETER_CHANGE; cnl: ") << pHex(&buf[11],1) << F(", peer: ") << pHex(&buf[12],4) << F(", pLst: ") << pHex(&buf[16],1) << F(", data: ") << pHex(&buf[17],b_len-16);

	} else if ((b_msgTp == 0x10) && (b_by10 == 0x06)) {
		Serial << F("INFO_ACTUATOR_STATUS; cnl: ") << pHex(&buf[11],1) << F(", status: ") << pHex(&buf[12],1) << F(", na: ") << pHex(&buf[13],1);
		if (b_len > 13) Serial << F(", rssi: ") << pHex(&buf[14],1);
		
	} else if ((b_msgTp == 0x11) && (b_by10 == 0x02)) {
		Serial << F("SET; cnl: ") << pHex(&buf[11],1) << F(", value: ") << pHex(&buf[12],1) << F(", rampTime: ") << pHex(&buf[13],2) << F(", duration: ") << pHex(&buf[15],2);

	} else if ((b_msgTp == 0x11) && (b_by10 == 0x03)) {
		Serial << F("STOP_CHANGE; cnl: ") << pHex(&buf[11],1);

	} else if ((b_msgTp == 0x11) && (b_by10 == 0x04) && (b_by11 == 0x00)) {
		Serial << F("RESET");

	} else if ((b_msgTp == 0x11) && (b_by10 == 0x80)) {
		Serial << F("LED; cnl: ") << pHex(&buf[11],1) << F(", color: ") << pHex(&buf[12],1);

	} else if ((b_msgTp == 0x11) && (b_by10 == 0x81) && (b_by11 == 0x00)) {
		Serial << F("LED_ALL; Led1To16: ") << pHex(&buf[12],4);
		
	} else if ((b_msgTp == 0x11) && (b_by10 == 0x81)) {
		Serial << F("LED; cnl: ") << pHex(&buf[11],1) << F(", time: ") << pHex(&buf[12],1) << F(", speed: ") << pHex(&buf[13],1);
		
	} else if ((b_msgTp == 0x11) && (b_by10 == 0x82)) {
		Serial << F("SLEEPMODE; cnl: ") << pHex(&buf[11],1) << F(", mode: ") << pHex(&buf[12],1);
		
	} else if ((b_msgTp == 0x12)) {
		Serial << F("HAVE_DATA");
		
	} else if ((b_msgTp == 0x3E)) {
		Serial << F("SWITCH; dst: ") << pHex(&buf[10],3) << F(", na: ") << pHex(&buf[13],1) << F(", cnl: ") << pHex(&buf[14],1) << F(", counter: ") << pHex(&buf[15],1);
		
	} else if ((b_msgTp == 0x3F)) {
		Serial << F("TIMESTAMP; na: ") << pHex(&buf[10],2) << F(", time: ") << pHex(&buf[12],2);
		
	} else if ((b_msgTp == 0x40)) {
		Serial << F("REMOTE; button: ") << pHex(buf[10] & 0x3F) << F(", long: ") << (buf[10] & 0x40 ? 1:0) << F(", lowBatt: ") << (buf[10] & 0x80 ? 1:0) << F(", counter: ") << pHex(buf[11]);
		
	} else if ((b_msgTp == 0x41)) {
		Serial << F("SENSOR_EVENT; button: ") <<pHex(buf[10] & 0x3F) << F(", long: ") << (buf[10] & 0x40 ? 1:0) << F(", lowBatt: ") << (buf[10] & 0x80 ? 1:0) << F(", value: ") << pHex(&buf[11],1) << F(", next: ") << pHex(&buf[12],1);
		
	} else if ((b_msgTp == 0x53)) {
		Serial << F("SENSOR_DATA; cmd: ") << pHex(&buf[10],1) << F(", fld1: ") << pHex(&buf[11],1) << F(", val1: ") << pHex(&buf[12],2) << F(", fld2: ") << pHex(&buf[14],1) << F(", val2: ") << pHex(&buf[15],2) << F(", fld3: ") << pHex(&buf[17],1) << F(", val3: ") << pHex(&buf[18],2) << F(", fld4: ") << pHex(&buf[20],1) << F(", val4: ") << pHex(&buf[21],2);
		
	} else if ((b_msgTp == 0x58)) {
		Serial << F("CLIMATE_EVENT; cmd: ") << pHex(&buf[10],1) << F(", valvePos: ") << pHex(&buf[11],1);
		
	} else if ((b_msgTp == 0x70)) {
		Serial << F("WEATHER_EVENT; temp: ") << pHex(&buf[10],2) << F(", hum: ") << pHex(&buf[12],1);

	} else {
		Serial << F("Unknown Message, please report!");
	}
	Serial << F("\n\n");		
	#endif
}
// - Storage Management ---------------------------------------------------------------------------------------------------
void    HM::initRegisters() {
	// check for magic number in eeprom to see if we have a first run
        while (!eeprom_is_ready());
	uint16_t tmagicNumber = eeprom_read_word(0);
	if (tmagicNumber != magicNumber) {
		
		#if defined(SM_DBG)														// some debug message
		Serial << F("first start detected, formating eeprom...\n");
		#endif
		
		for (uint16_t l = 0; l <  sizeof(s_EEPROM); l++) {						// step through the bytes of eeprom
			setEEpromByte(l, 0);												// and write a 0
		}
		
		#if defined(SM_DBG)														// some debug message
		Serial << F("done\n");
		#endif

                while (!eeprom_is_ready());
		eeprom_write_word(0,magicNumber);										// to check next time if we have already our structure in eeprom
	}
	
	// load default settings to eeprom if firstLoad is defined
	#if defined(firstLoad)
	uint16_t regPtr, peerPtr;
	mainSettings(&regPtr,&peerPtr);
	eeprom_write_block((const void*)regPtr,(void*)&ee->regs,sizeof(ee->regs));
	eeprom_write_block((const void*)peerPtr,(void*)&ee->peerdb,sizeof(ee->peerdb));
	#endif

	// read the peer database back
        while (!eeprom_is_ready());
	getEEpromBlock((uint16_t)&ee->peerdb, sizeof(peerdb), &peerdb);				// read back peer database
	#if defined(SM_DBG)															// some debug message
	Serial << F("Loading PeerDB, starts: ") << (uint16_t)&ee->peerdb << F(", size of: ") << sizeof(ee->peerdb) << '\n';
	#endif

	getMainChConfig();															// fill the structure of main channel configs
};

// slice table functions
uint8_t HM::getSliceDetail(uint8_t cnl, uint8_t lst, uint8_t *peer, s_slcVar *sV) {
	uint8_t peerIdx, cType, cnt;												// size the variables
	uint16_t listSliceIdx = 0;
	uint32_t peerL =  *(long*)&peer[0];											// convert peer from byte array to long

	// request must be valid and peer must exist
	if (cnl >= devDef.nbrChannels) return 0;									// channel out of range, end
	if ((peerL == 0) && (lst >= 3)) return 0;									// empty peer is not valid for List3 or List4
	if ((cnl == 0) && (lst !=0)) return 0;										// channel 0 has only a list0

	if (lst < 3) peerIdx = 0;													// List0 & List1 needs no index
	else peerIdx = getIdxByPeer(cnl,peer);										// get the peer index of respective peer
	//Serial << "pI:" << peerIdx << ", cnl:" << cnl << '\n';
	if (peerIdx == 0xff) return 0;												// peer failure

	cType = devDef.chDefType[cnl].type;											// is the request for the device or for a channel
	for (cnt = 0; cnt < listTypeDef[cType].nbrLists; cnt++) {					// find respective list number
		if (listTypeDef[cType].type[cnt].ListNo == lst) break;
		listSliceIdx += listTypeDef[cType].type[cnt].nbrOfSlice;				// remember the appropriate listSliceIndex
	}
	
	if (listTypeDef[cType].nbrLists <= cnt) return 0;							// list not found
	if (listTypeDef[cType].type[cnt].nbrPeers < peerIdx) return 0;				// peer out of range

	// find slice and read variables
	sV->slcPtr = devDef.chDefType[cnl].sliceIdx + listSliceIdx +
	peerIdx*listTypeDef[cType].type[cnt].nbrOfSlice;							// calculate slice idx incl. peer idx

	sV->phyAddr = sliceStr[sV->slcPtr].phyAddr;									// get physical address
	
	sV->slcLen = listTypeDef[cType].type[cnt].nbrOfSlice;						// define end slice
	sV->phyLen = 0;																// len to 0 because we want to add current slices
	for (uint8_t i = 0; i < sV->slcLen; i++) {									// step through slices
		sV->phyLen += sliceStr[sV->slcPtr+i].nbrBytes;							// add amount of bytes
	}

	#if defined(SM_DBG)															// some debug message
	Serial << F("getSliceDetail, slcPtr: ") << sV->slcPtr << F(", slcLen: ") << sV->slcLen
	<< F(", phyAddr: ") << sV->phyAddr << F(", phyLen: ") << sV->phyLen << F(", peerIdx: ") << peerIdx << '\n';
	#endif

	return 1;
}
uint8_t HM::doesListExist(uint8_t cnl, uint8_t lst) {
	// check if a list exist
	uint8_t cType, cnt;															// size variables
	cType = devDef.chDefType[cnl].type;											// get channel device type by channel
	for (cnt = 0; cnt < listTypeDef[cType].nbrLists; cnt++) {					// find respective list number
		if (listTypeDef[cType].type[cnt].ListNo == lst) break;					// if we have found list4 then break
	}
	//Serial << "cnl: " << cnl << ", cType: " << cType << ", lst: " << lst << ", cnt: " << cnt << '\n';
	if (listTypeDef[cType].nbrLists <= cnt) return 0;							// list not found
	else return 1;																// list found
}
uint8_t HM::getRegList(uint8_t slcPtr, uint8_t slcLen, uint8_t *buf) {
        #if defined(SM_DBG)
	uint8_t *x = buf;															// needed for debug message
        #endif

	slcLen += slcPtr;															// calculate slice len
	
	uint8_t msgLen = 0;															// counter for bytes written in message
	for (uint8_t i = slcPtr; i < slcLen; i++) {									// count through slice table
		uint16_t addr = sliceStr[i].regAddr;									// position the pointer on the slice string
		
		for (uint8_t j = 0; j < sliceStr[i].nbrBytes; j++) {					// count through slice
			*(buf++) = addr++;													// write to buffer
			msgLen++;															// increase message counter
		}
	}

	#if defined(SM_DBG)															// some debug message
	Serial << F("getReg, len: ") << msgLen << F(", data: ") << pHex(&x[0],msgLen) << '\n';
	#endif

	return msgLen;																// return the message len
}
void HM::getMainChConfig(void) {
	uint8_t ret, peer[] = {0xff,0xff,0xff,0x00};								// some declarations
	s_slcVar sV;
	
	// get cnl0 list0 for internal and external use
	ret = getSliceDetail(0, 0, (uint8_t*) &broadCast, &sV);
	getEEpromBlock(sV.phyAddr+(uint16_t)&ee->regs, sV.phyLen, &regDev);
	
	// step through the channels and lists and load registers
	uint8_t cnt = 0;
	uint16_t *mc = (uint16_t*)mcConfPtr;
	for (uint8_t i = 0; i <= maxChannel; i++) {									// count through the channel
		for (uint8_t j = 0; j <= 7; j++) {										// count through the lists, 7 should be the max
			if (!getSliceDetail(i, j, &peer[0], &sV)) continue;					// get the slice details, if empty then next
			if (j == 3) l3Ptr[i] = mc[cnt];										// remember list3 pointer
			//if (j == 4) l4Ptr[i] = mc[cnt];										// remember list4 pointer
			getEEpromBlock(sV.phyAddr+(uint16_t)&ee->regs, sV.phyLen, (void*)mc[cnt]);
			cnt++;																// increase pointer to ptr list
		}
	}
}
void HM::getList3ByPeer(uint8_t cnl, uint8_t *peer) {
	s_slcVar sV;																// some declarations
	
	uint8_t ret = getSliceDetail(cnl, 3, peer, &sV);							// get cnl list3 for external use
	if (ret) getEEpromBlock(sV.phyAddr+(uint16_t)&ee->regs, sV.phyLen, (void*)l3Ptr[cnl]);

	#if defined(SM_DBG)															// some debug message
	Serial << F("Loading list3 for cnl: ") << cnl << F(", peer: ") << pHex(peer,4) << '\n';
	#endif
}

// message generation for TRX868 module
uint8_t HM::getListForMsg2(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf) {
	// return 0     = no data, junk out of range
	//        3-254 = chars returned
	//        0xff  = input params unknown

	const uint8_t bytesPerJunk = 8;														// how many bytes should one junk deliver
	
	static uint8_t tcnl, tlst, msgPtr = 0;										// size variables
	static uint32_t tpeerL, peerL;
	#if defined(SM_DBG)
	uint8_t *tbuf = buf;														// remember the start position of buffer
        #endif
	static s_slcVar sV;
	
	// check if we are complete, check mark is the physical len of the data string
	if (msgPtr > sV.phyLen) return msgPtr = 0;										// we are completely through, stop action
	
	// check to load the storage details only one time
	peerL = *(long*)&peer[0];													// convert byte array to long
	if ((tcnl != cnl) || (tlst != lst) || (tpeerL != peerL)) msgPtr = 0;		// request had changed, start from beginning
	tcnl = cnl; tlst = lst; tpeerL = peerL;										// store for next check

	if (msgPtr == 0) {															// only the first time we have to get the details
		if (!getSliceDetail(cnl, lst, peer, &sV)) return 0xff;
		sV.slcLen += sV.slcPtr;													// calculate slice len
	}

	#if defined(SM_DBG)															// some debug message
	Serial << F("\ngetListForMsg2, msgPtr: ") << msgPtr << F(" of ") << sV.phyLen
	<< F(", slcPtr: ") << sV.slcPtr << ", slcEnd: " << sV.slcLen << '\n';
	#endif

	// go through the slices and take only the bytes needed
	uint8_t msgCnt = 0, bInMsg = 0;												// counter the bytes
	
	for (uint8_t i = sV.slcPtr; i < sV.slcLen; i++) {							// count through slice table
		if (bInMsg >= bytesPerJunk) break;										// step out if we have enough bytes
		uint8_t addr = sliceStr[i].regAddr;										// remember the address byte from slice table
		uint16_t dataPtr = sliceStr[i].phyAddr + (uint16_t)&ee->regs;			// remember the physical address
		
		for (uint8_t j = 0; j < sliceStr[i].nbrBytes; j++) {					// count through slice
			msgCnt++;															// increase message counter
			if (msgCnt > msgPtr) {												// write only to buffer if we need that bytes
				bInMsg++;														// increase the byte counter
				*(buf++) = addr;												// add the address byte
				*(buf++) = getEEpromByte(dataPtr);								// add the data byte from eeprom
			}
			if (bInMsg >= bytesPerJunk) break;									// step out if we have enough bytes
			addr++;																// increase addr counter accordingly
			dataPtr++;															// increase the pointer to the eeprom location
		}
	}
	msgPtr = msgCnt;															// set the msgPtr for next try
	
	// add the 00 00 as termination
	if ((msgPtr == sV.phyLen) && (bInMsg < bytesPerJunk)) {						// if we have delivered all bytes and there is some space in the junk
		*(buf++) = 0;															// add a 0 on end for termination
		*(buf++) = 0;															// add a 0 on end for termination
		msgPtr++;																// increase msgPtr to step out next try
		bInMsg++;																// increase the byte counter to deliver the right amount of bytes
	}
	bInMsg = bInMsg << 1;														// *2 because we add addr and data byte
	
	#if defined(SM_DBG)															// some debug message
	Serial << F("getListForMsg2, len: ") << bInMsg << F(", data: ") << pHex(&tbuf[0],bInMsg) << "\n\n";
	#endif
	
	return bInMsg;																// return accordingly
}
uint8_t HM::getListForMsg3(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf) {
	return 0;
}
uint8_t HM::setListFromMsg(uint8_t cnl, uint8_t lst, uint8_t *peer, const uint8_t *buf, uint8_t len) {
	uint8_t aLen; // size variables
	#if defined(SM_DBG)	
        const uint8_t* tbuf = buf;
	#endif													
	s_slcVar sV;

	//Serial << "we: " << conf.wrEn << ", cnl: " << cnl << ", lst: " << lst << ", peer: " << pHex(peer,4) << '\n';
	#if defined(SM_DBG)															// some debug message
	Serial << "pl: " << pHex(tbuf,len)  << '\n';
	#endif

	// get the slice details and the address list
	if (!getSliceDetail(cnl, lst, peer, &sV)) return 0;
	uint8_t addrStr[sV.phyLen];													// size the buffer for the address string
	
	aLen = getRegList(sV.slcPtr, sV.slcLen, addrStr);							// get the address string
	if (aLen == 0) return 0;													// if string is empty we can leave
	//Serial << "aS: " << pHex(addrStr,sV.phyLen) << '\n';

	#if defined(SM_DBG)															// some debug message
	Serial << F("setListFromMsg, len: ") << aLen << F(", data: ") << pHex(&addrStr[0],aLen) << "\nwrite: ";
	#endif

	// step through the buffer bytes and search for the right address
	uint16_t dataPtr;
	for (uint8_t i = 0; i < len; i+=2) {										// count through input string
		void *x = memchr(addrStr, buf[i], sV.phyLen);							// search the character in the address string
		if ((uint16_t)x == 0) continue;											// if we got no result try next
		dataPtr = (uint16_t)x-(uint16_t)addrStr;								// calculate the respective address in list 

		//Serial << "search: " << pHex(buf[i]) << ", aStr: " << pHex(addrStr,sV.phyLen) << ", fp: " << dataPtr << '\n';
		setEEpromByte(dataPtr+(uint16_t)&ee->regs+sV.phyAddr,buf[i+1]);			// write the byte
		
		#if defined(SM_DBG)														// some debug message
		Serial << pHex(buf[i]) << ":" << pHex(buf[i+1]) << " ";
		#endif
	}

	#if defined(SM_DBG)															// some debug message
	Serial << "\n\n";
	#endif

	// reread of channel config will be done when we got the message to end setup
	//if ((lst <= 1) || (lst >= 5)) getMainChConfig();							// reread main channel config
	return 1;
}
uint8_t HM::getPeerListForMsg(uint8_t cnl, uint8_t *buf) {
	const uint8_t bytesPerJunk = 16;														// how many bytes should one junk deliver
	if (cnl > maxChannel) return 0xff;											// if channel out of range, return

	uint8_t *t, cnt=0;															// size variables
	static uint8_t tcnl, tPtr;
	cnl--;																		// adjust channel while peer database starts with 0
	
	if (tPtr > peermax[cnl]) return tPtr = 0;									// tPtr has exceed limit, assumption we are completely through, stop action
	if (cnl != tcnl) tPtr = 0;													// channel has changed, assume we got a new request
	tcnl = cnl;																	// remember channel for next try
	
	while (tPtr < peermax[cnl]) {												// step through the peers of specific channel
		if (peerdb[cnl][tPtr]) {												// only step in while some content available
			t = (uint8_t*)&peerdb[cnl][tPtr++];									// pointer to the content in database
			memcpy(buf+cnt,t,4);												// copy to buffer
			cnt+=4;																// increase buffer for next try
			if (cnt >= bytesPerJunk) return cnt;								// take care of max payload
		} else tPtr++;															// if nothing in, increase the pointer only
	}
	memset(buf+cnt,0,4);														// add termination bytes
	tPtr++;
	return cnt+=4;																// all done, waiting for next request
}

// peerdb handling; add, remove and search functions
uint8_t HM::addPeerFromMsg(uint8_t cnl, uint8_t *peer) {
	if (cnl > maxChannel) return 0xff;											// if channel out of range, return

	uint8_t ret, tPeer[4];														// size variables
	
	// copy first peer to database, if everything is ok we got a 1 back
	memcpy(tPeer,peer,4);														// copy the peer in a variable
	ret = addPeerToDB(cnl,tPeer);
	#if defined(SM_DBG)															// some debug message
	Serial << F("addPeerFromMsg, cnl: ") << cnl << ", ret: " << ret << '\n';
	#endif

	// check if we have to add another peer, if not return the status of adding the first peer
	if (peer[4] == 0 || peer[3] == peer[4]) {
            if (ret) {
                loadDefaultRegset(cnl, peer, false, 0); 
                while (!eeprom_is_ready());
                getMainChConfig();
            }
            return ret;
        }
        
        if (ret) {
            loadDefaultRegset(cnl, peer, true, 0);
        }

	// if we are here we have to copy a second peer to database, if everything is ok we got a 1 back
	tPeer[3] = peer[4];															// copy the second channel to the peer
	ret = addPeerToDB(cnl,tPeer);

        if (ret) {
            loadDefaultRegset(cnl, peer, true, 1);
        }
        
        while (!eeprom_is_ready());
        getMainChConfig();
        
	#if defined(SM_DBG)															// some debug message
	Serial << F("addPeerFromMsg, cnl: ") << cnl << ", ret: " << ret << '\n';
	#endif
	return ret;																	// return the status of the second add	
}
uint8_t HM::removePeerFromMsg(uint8_t cnl, uint8_t *peer) {
	if (cnl > maxChannel) return 0xff;											// if channel out of range, return

	uint8_t idx1, idx2, tPeer[4];												// size variables

	// remove both peers from the database; we will not check if both are existing
	// if we found a valid index, we will delete the peer by writing the broad cast variable in the slot
	memcpy(tPeer,peer,4);														// copy the peer in a variable
	idx1 = getIdxByPeer(cnl, tPeer);											// get the idx of the first peer
	if (idx1 != 0xff) setEEpromBlock((uint16_t)&ee->peerdb[cnl-1][idx1],4,(uint8_t*) &broadCast);

	tPeer[3] = peer[4];															// change the peer channel
	idx2 = getIdxByPeer(cnl, tPeer);											// get the idx of the second peer
	if (idx2 != 0xff) setEEpromBlock((uint16_t)&ee->peerdb[cnl-1][idx2],4,(uint8_t*) &broadCast);


	#if defined(SM_DBG)															// some debug message
	Serial << F("removePeerFromMsg, cnl: ") << cnl << F(", pIdx1: ") << idx1 << F(", pIdx2: ") << idx2 << '\n';
	#endif

	getEEpromBlock((uint16_t)&ee->peerdb, sizeof(peerdb), &peerdb);				// read back peer database
	return 1;																	// every thing went ok,
}
uint8_t HM::getCnlByPeer(uint8_t *peer) {
	if (memcmp(peer,broadCast,4) == 0) return 0;								// return failure while peer is empty

	for (uint8_t i=0; i < maxChannel; i++) {									// step through the channels
		for (uint8_t j=0; j < peermax[i]; j++) {								// step through the peers of channel i
			if (memcmp((uint8_t*)&peerdb[i][j],peer,4) == 0) {
				//Serial << "x i:" << i << ", j:" << j << '\n';
				return i+1;		// return 1 if we found something
			}
		}
	}
	return 0;																	// found nothing, return 0
}
uint8_t HM::getIdxByPeer(uint8_t cnl, uint8_t *peer) {
	uint8_t tPeer[3] = {0xff,0xff,0xff};
	if (cnl > maxChannel) return 0xff;											// check against max channels

	cnl--;																		// adjust channel due to database start with 0
	for (uint8_t j=0; j < peermax[cnl]; j++) {									// step through the peers of channel i
		if (memcmp((uint8_t*)&peerdb[cnl][j],peer,4) == 0) return j;			// return idx if we found something
	}

	if (memcmp(peer,tPeer,3) == 0) {											// dummy peer, FFFFFF01 means index 1
		if (peer[3] >= peermax[cnl]) return 0xff;								// if index out of range, return failure
		else return peer[3];													// otherwise give index
	}

	return 0xff;																// nothing found, return failure
}
uint8_t HM::getPeerByIdx(uint8_t cnl, uint8_t idx, uint8_t *peer) {
	if (cnl > maxChannel) return 0;												// check against max channels
	if (idx >= peermax[cnl-1]) return 0;										// check against max peer slots

	cnl--;																		// adjust channel due to database start with 0
	memcpy(peer,(uint8_t*)&peerdb[cnl][idx],4);									// copy peer from peerdb
	return 1;																	// everything should be fine, return 1
}
uint8_t HM::getFreePeerSlot(uint8_t cnl) {
	cnl--;																		// adjust channel due to database start with 0
	for (uint8_t j=0; j < peermax[cnl]; j++) {									// step through the peers of channel
		if (memcmp((uint8_t*)&peerdb[cnl][j],broadCast,4) == 0) return j;		// return idx if we found a free slot
	}
	return 0xff;																// otherwise return failure
}
uint8_t HM::countFreePeerSlot(uint8_t cnl) {
	uint8_t counter = 0;														// size counter variable and set to 0
	cnl--;																		// adjust channel due to database start with 0
	for (uint8_t j=0; j < peermax[cnl]; j++) {									// step through the peers of channel
		if (memcmp((uint8_t*)&peerdb[cnl][j],broadCast,4) == 0) counter++;		// increase counter if we found a free slot
	}
	return counter;																// otherwise return failure
}

uint8_t HM::loadDefaultRegset(uint8_t cnl, uint8_t *peer, boolean dual, uint8_t idx) {
        uint8_t type = devDef.chDefType[cnl].type;  // get channel type
        if (default_regChans_dev[type].regChan_len == 0) return 0;  // No default. Nothing to do
        
        // get the slice details and the address list
	s_slcVar sV;
	if (!getSliceDetail(cnl, default_regChans_dev[type].lst, peer, &sV)) return 0;
        
        Serial << "loadDefaultRegset: cnl=" << cnl << " dual: " << dual << " regChan_len: " << default_regChans_dev[type].regChan_len << " lst: " << default_regChans_dev[type].lst << " idx: " << idx << '\n';
        const uint8_t* regset;
        if (dual) {
          if (idx == 0) {
            regset = default_regChans_dev[type].default_regChan_dual_1;
          } else {
            regset = default_regChans_dev[type].default_regChan_dual_2;
          }
        } else {
          regset = default_regChans_dev[type].default_regChan_single;
        }
        //eeprom_write_block((const void*)regset,(void*)&sV.phyAddr,sV.phyLen);
        setEEpromBlock(sV.phyAddr+(uint16_t)&ee->regs, sV.phyLen, (void*)regset);
//        setEEpromBlock(sV.phyAddr+(uint16_t)&ee->regs, sV.phyLen, (void*)regset);
        //setEEpromBlock
        return 1;
}

uint8_t HM::addPeerToDB(uint8_t cnl, uint8_t *peer) {
	// check if peer is already known
	uint8_t tCnl = getIdxByPeer(cnl,peer);
	//Serial << ",tcnl: " << tCnl << ", peer: " << pHex(peer,4) << '\n';
	if (tCnl < 0xFF) {
		//Serial << "already exist\n";
		return 0;													// peer already exist
	}
	// check if we have a free slot and add to eeprom
	uint8_t idx = getFreePeerSlot(cnl);											// find a free slot
	if (idx == 0xff) return 0;													// no free slot 
	//Serial << "cnl: " << cnl << ", idx: " << idx << ", peer: " << pHex(peer,4) << '\n';
	setEEpromBlock((uint16_t)&ee->peerdb[cnl-1][idx],4,peer);					// write peer to eeprom
	
	// reload database
	getEEpromBlock((uint16_t)&ee->peerdb, sizeof(peerdb), &peerdb);				// read back peer database
	return 1;																	// every thing went ok,
}
	
// to check incoming messages if sender is known
uint8_t HM::isPeerKnown(uint8_t *peer) {
	for (uint8_t i=0; i < maxChannel; i++) {									// step through the channels
		for (uint8_t j=0; j < peermax[i]; j++) {								// step through the peers of channel i
			if (memcmp((uint8_t*)&peerdb[i][j],peer,3) == 0) return 1;			// return 1 if we found something
		}
	}
	return 0;																	// found nothing, return 0
}
uint8_t HM::isPairKnown(uint8_t *pair) {
	if (memcmp(regDev.pairCentral, broadCast, 3) == 0) return 1;				// return 1 while not paired
	
	if (memcmp(regDev.pairCentral, pair, 3) == 0) return 1;						// check against regDev
	else return 0;																// found nothing, return 0
}

// pure eeprom handling, i2c must implemented
uint8_t HM::getEEpromByte(uint16_t addr) {
	// todo: lock against writing
	// todo: extend range for i2c eeprom
	return eeprom_read_byte((uint8_t*)addr);
}
void HM::setEEpromByte(uint16_t addr, uint8_t payload) {
	// todo: lock against reading
	// todo: extend range for i2c eeprom
	return eeprom_write_byte((uint8_t*)addr,payload);
}
void HM::getEEpromBlock(uint16_t addr,uint8_t len,void *ptr) {
	// todo: lock against reading
	// todo: extend range for i2c eeprom
	eeprom_read_block((void*)ptr,(const void*)addr,len);
}
void HM::setEEpromBlock(uint16_t addr,uint8_t len,void *ptr) {
	// todo: lock against reading
	// todo: extend range for i2c eeprom
	eeprom_write_block((const void*)ptr,(void*)addr,len);
}


//- -----------------------------------------------------------------------------------------------------------------------
//- button key functions ---------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
void BK::config(uint8_t Cnl, uint8_t Pin, uint16_t TimeOutShortDbl, uint16_t LongKeyTime, uint16_t TimeOutLongDdbl, void tCallBack(uint8_t, uint8_t)) {

	// settings while setup
	pinMode(Pin, INPUT_PULLUP);													// setting the pin to input mode
	toShDbl = TimeOutShortDbl;													// minimum time to be recognized as a short key press
	lngKeyTme  = LongKeyTime;													// time key should be pressed to be recognized as a long key press
	toLoDbl = TimeOutLongDdbl;													// maximum time between a double key press
	callBack = tCallBack;														// call back address for button state display
	
	// default settings
	pin = Pin;
	cFlag = 0;																	// no need for the poll routine at the moment
	cStat = 1;																	// active low, means last state should be active to get the next change
	lStat = 1;
	dblLo = 0;																	// counter for a double low
	rptLo = 0;																	// counter for repeated low
	
	// setting the interrupt and port mask
	// http://www.kriwanek.de/arduino/grundlagen/183-mehrere-pin-change-interrupts-verwenden.html
	volatile uint8_t* pcicr = digitalPinToPCICR(Pin);
	*pcicr |= (1 << digitalPinToPCICRbit(Pin));
	volatile uint8_t* pcmsk = digitalPinToPCMSK(Pin);
	*pcmsk |= (1 << digitalPinToPCMSKbit(Pin));

	// load the respective pin register to mask out in interrupt routine
	uint8_t pinPort = digitalPinToPort(Pin)-1;									// get the respective port to the given pin
	pci.lPort[pinPort] = *portInputRegister(pinPort+1) & *pcmsk;				// store the port input byte for later comparison
	pci.pAddr[pinPort] = (uint8_t*)portInputRegister(pinPort+1);				// store the address of the port input register to avoid PGM read in the interrupt
	
	// set index and call back address for interrupt handling
	idx = Cnl;																	// set the index in the interrupt array
	pci.ptr[idx] = this;														// set the call back address
	pci.idx[idx] = (pinPort << 8) + (1 << digitalPinToPCMSKbit(Pin));			// calculate and set the index number for faster finding in the interrupt routine
	//Serial << "pin:" << tPin << ", idx:" << pci.idx[pci.nbr] << ", prt:" << pinPort << ", msk:" << (1 << digitalPinToPCMSKbit(tPin)) << '\n';
}
void BK::poll() {
	for (uint8_t i = 0; i < maxInt; i++) {
		if (pci.ptr[i]) {
			BK *p = pci.ptr[i];
			p->poll_btn();
		}
	}
}
void BK::poll_btn() {
	// possible events of this function:
	// 0 - short key press
	// 1 - double short key press
	// 2 - long key press
	// 3 - repeated long key press
	// 4 - end of long key press
	// 5 - double long key press
	// 6 - time out for double long
	
	if (cFlag == 0) return;														// no need for do any thing
	if (cTime > millis()) return;												// for debouncing and timeout issues 
	
	if ((cStat == 1) && (lStat == 1)) {											// timeout
	// only timeouts should happen here
		if ((dblLo) && (kTime + toLoDbl <= millis())) {							// timeout for double long reached
			dblLo = 0;															// no need for check against
			//Serial << "dbl lo to\n";
			callBack(idx,6);													// raise timeout for double long
		}
		if ((dblSh) && (kTime + toShDbl <= millis())) {							// timeout for double short reached
			dblSh = 0;															// no need for check against
			//Serial << "dbl sh to\n";
		}

		if ((dblLo == 0) && (dblSh == 0)) cFlag = 0;							// no need for checking again
		if (dblLo) cTime = millis() + toLoDbl;									// set the next check time
		if (dblSh) cTime = millis() + toShDbl;									// set the next check time
	
	} else if ((cStat == 1) && (lStat == 0)) {									// key release
	// coming from a short or long key press, end of long key press by checking against rptLo

		if (rptLo) {															// coming from a long key press
			rptLo = 0;															// could not be repeated any more
			//Serial << "end lo\n";
			callBack(idx,4);													// end of long key press
			
		} else if (dblSh) {														// double short was set
			dblSh = 0;															// no need for this flag anymore
			//Serial << "dbl sh\n";
			callBack(idx,1);													// double short key press

		} else if (kTime + lngKeyTme > millis()) {								// short key press
			dblSh = 1;															// next time it could be a double short
			//Serial << "sh\n";
			if (!toShDbl) callBack(idx,0);										// short key press
		} 
		if ((dblSh) && (toShDbl)) cTime = millis() + toShDbl;					// set the next check time
		if (dblLo) cTime = millis() + toLoDbl;									// set the next check time
		kTime = millis();														// set variable to measure against
		lStat = cStat;															// remember last key state
		cFlag = 1;																// next check needed

	} else if ((cStat == 0) && (lStat == 1)) {
	// key is pressed just now, set timeout 
		kTime = millis();														// store timer
		cTime = millis() + lngKeyTme;											// set next timeout
		lStat = cStat;															// remember last key state
		cFlag = 1;																// next check needed
		
	} else if ((cStat == 0) && (lStat == 0)) {
	// next check time while long key press or a repeated long key press
	// if it is a long key press, check against dblLo for detecting a double long key press
		if (rptLo) {															// repeated long detect
			dblLo = 0;															// could not be a double any more
			cTime = millis() + lngKeyTme;										// set next timeout
			//Serial << "rpt lo\n";
			callBack(idx,3);													// repeated long key press
			
		} else if (dblLo) {														// long was set last time, should be a double now
			rptLo = 0;															// could not be a repeated any more
			dblLo = 0;															// could not be a double any more
			cFlag = 0;															// no need for jump in again
			//Serial << "dbl lo\n";
			callBack(idx,5);													// double long key press
			
		} else {																// first long detect
			dblLo = 1;															// next time it could be a double
			rptLo = 1;															// or a repeated long
			cTime = millis() + lngKeyTme;										// set next timeout
			//Serial << "lo\n";
			callBack(idx,2);													// long key press
			
		}
	}
}


//- -----------------------------------------------------------------------------------------------------------------------
//- relay functions -------------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
// public function for setting the module
void RL::config(uint8_t cnl, void msgCallBack(uint8_t, uint8_t, uint8_t), void adjRlyCallback(uint8_t cnl, uint8_t tValue), HM *statCallBack, uint8_t minDelay, uint8_t randomDelay) {
	// store config settings in class
	mDel = minDelay;															// remember minimum delay for sending the status
	rDel = (randomDelay)?randomDelay:1;											// remember random delay for sending the status
	cbS = statCallBack;															// call back address for sending status and ACK
	cbM = msgCallBack;
        adjRlyCb = adjRlyCallback;	

	prl.ptr[prl.nbr++] = this;													// register inxtStatnce in struct
	cnlAss = cnl;																// stores the channel for the current instance
	curStat = 6;																// set relay status to off
	adjRly(0);																	// set relay to a defined status
}

// public functions for triggering some action
void RL::trigger11(uint8_t val, uint8_t *rampTime, uint8_t *duraTime) {
	// {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}

	rTime = (uint16_t)rampTime[0]<<8 | (uint16_t)rampTime[1];					// store ramp time
	dTime = (duraTime)?((uint16_t)duraTime[0]<<8 | (uint16_t)duraTime[1]):0;	// duration time if given

	if (rTime) nxtStat = (val == 0)?4:1;										// set next status
	else nxtStat = (val == 0)?6:3;
	
	lastTrig = 11;																// remember the trigger
	rlyTime = millis();															// changed some timers, activate poll function
	cbS->sendACKStatus(cnlAss,val,((nxtStat==1)||(nxtStat==4))?0x40:0);			// send an status ACK 
	
	#if defined(RL_DBG)															// some debug message
	Serial << F("RL:trigger11, val:") << val << F(", nxtS:") << nxtStat << F(", rampT:") << rTime << F(", duraT:") << dTime << '\n';
	#endif
}
void RL::trigger41(uint8_t lngIn, uint8_t val, void *plist3) {
	lastTrig = 41;																// set trigger
	rlyTime = millis();															// changed some timers, activate poll function
}
void RL::trigger40(uint8_t lngIn, uint8_t cnt, void *plist3) {
	s_peer_regChan_actor* srly = (s_peer_regChan_actor*)plist3;														// copy list3 to pointer
	static uint8_t rCnt;														// to identify multi execute
	
	// check for repeated message	
	if ((lngIn) && (srly->lgMultiExec == 0) && (cnt == rCnt)) return;			// trigger was long
	if ((lngIn == 0) && (cnt == rCnt)) return;									// repeated instruction
	rCnt = cnt;																	// remember message counter

	// fill the respective variables
	uint8_t actTp = (lngIn)?srly->lgActionType:srly->shActionType;				// get actTp = {off=>0,jmpToTarget=>1,toggleToCnt=>2,toggleToCntInv=>3}

	if (actTp == 0) {															// off
		nxtStat = 0; // do not change state
	} else if ((actTp == 1) && (lngIn == 1)) {									// jmpToTarget
		// SwJtOn {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
		if      (curStat == 6) nxtStat = srly->lgSwJtOff;						// currently off
		else if (curStat == 3) nxtStat = srly->lgSwJtOn;						// on
		else if (curStat == 4) nxtStat = srly->lgSwJtDlyOff;					// delay off
		else if (curStat == 1) nxtStat = srly->lgSwJtDlyOn;						// delay on
		OnDly   = srly->lgOnDly;												// set timers
		OnTime  = srly->lgOnTime;
		OffDly  = srly->lgOffDly;
		OffTime = srly->lgOffTime;

	} else if ((actTp == 1) && (lngIn == 0)) {									// jmpToTarget
		if      (curStat == 6) nxtStat = srly->shSwJtOff;						// currently off
		else if (curStat == 3) nxtStat = srly->shSwJtOn;						// on
		else if (curStat == 4) nxtStat = srly->shSwJtDlyOff;					// delay off
		else if (curStat == 1) nxtStat = srly->shSwJtDlyOn;						// delay on
		OnDly   = srly->shOnDly;												// set timers
		OnTime  = srly->shOnTime;
		OffDly  = srly->shOffDly;
		OffTime = srly->shOffTime;

	} else if (actTp == 2) {													// toogleToCnt, if tCnt is even, then next state is on
		nxtStat = (cnt % 2 == 0)?3:6;											// even - relay dlyOn, otherwise dlyOff
		OnDly   = 0; OnTime  = 255; OffDly  = 0; OffTime = 255;					// set timers
		
	} else if (actTp == 3) {													// toggleToCntInv, if tCnt is even, then next state is off, while inverted
		nxtStat = (cnt % 2 == 0)?6:3;											// even - relay dlyOff, otherwise dlyOn
		OnDly   = 0; OnTime  = 255; OffDly  = 0; OffTime = 255;					// set timers
	}
	lastTrig = 40;																// set trigger
	rlyTime = millis();															// changed some timers, activate poll function

	#if defined(RL_DBG)															// some debug message
	Serial << F("RL:trigger40, curS:") << curStat << F(", nxtS:") << nxtStat << F(", OnDly:") << OnDly << F(", OnTime:") << OnTime << F(", OffDly:") << OffDly << F(", OffTime:") << OffTime << '\n';
	#endif
	
	cbS->sendACKStatus(cnlAss,getRly(),((nxtStat==1)||(nxtStat==4))?0x40:0);
}
void RL::sendStatus(void) {
	if (cbS) cbS->sendInfoActuatorStatus(cnlAss,getRly(),getStat());			// call back
}

// public poll function to poll relay and delayed status messages
void RL::poll(void) {
	if (prl.nbr == 0) return;													// no inxtStatnce listed
	for (uint8_t i = 0; i < prl.nbr; i++) {										// step through inxtStatnces
		prl.ptr[i]->poll_rly();													// and poll the relay
		prl.ptr[i]->poll_cbd();													// and poll the call back timer
	}
}

// private functions for setting relay and getting current status
void RL::adjRly(uint8_t tValue) {

	#if defined(RL_DBG)															// some debug message
	Serial << F("RL:adjRly, curS:") << curStat << F(", nxtS:") << nxtStat << '\n';
	#endif

        adjRlyCb(cnlAss, tValue);

	cbsTme = millis() + ((uint32_t)mDel*1000) + random(((uint32_t)rDel*1000));	// set the timer for sending the status
}
uint8_t RL::getRly(void) {
	// curStat could be {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
	if ((curStat == 1) || (curStat == 3)) return 0xC8;
	if ((curStat == 4) || (curStat == 6)) return 0x00;
        return 0x00; // Default
}
uint8_t RL::getStat(void) {
	// curStat could be {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
	return (rlyTime > 0)?0x40:0x00;
}

// private function for polling the relay and sending delayed status message
void RL::poll_rly(void) {
	if ((rlyTime == 0) || (rlyTime > millis())) return;							// timer set to 0 or time for action not reached, leave
	rlyTime = 0;																// freeze per default

	// set relay - {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
	if (nxtStat == 3 && curStat != 3) {															// set relay on
		adjRly(1); curStat = 3;													// adjust relay, status will send from adjRly()

	} else if (nxtStat == 6 && curStat != 6) {													// set relay off
		adjRly(0); curStat = 6;													// adjust relay, status will send from adjRly()
	}
	
	// adjust nxtStat for trigger11 - {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
	if (lastTrig == 11) {
		if (nxtStat == 1) {														// dlyOn -> on
			nxtStat = 3;														// next status is on
			rlyTime = millis() + intTimeCvt(rTime);								// set respective timer
		
		} else if ((nxtStat == 3) && (dTime > 0)) {								// on - > off
			nxtStat = 6;														// next status is off
			rlyTime = millis() + intTimeCvt(dTime);								// set the respective timer
		}
	}
		
	// adjust nxtStat for trigger40 - {no=>0,dlyOn=>1,on=>3,dlyOff=>4,off=>6}
	if (lastTrig == 40) {
		if        (nxtStat == 1) {
			nxtStat = 3;
			rlyTime = millis() + byteTimeCvt(OnDly);

		} else if ((nxtStat == 3) && (OnTime < 255)) {
			nxtStat = 4;
			if (OnTime) rlyTime = millis() + byteTimeCvt(OnTime);

		} else if (nxtStat == 4) {
			nxtStat = 6;
			rlyTime = millis() + byteTimeCvt(OffDly);

		} else if ((nxtStat == 6) && (OffTime < 255)) {
			nxtStat = 1;
			if (OffTime) rlyTime = millis() + byteTimeCvt(OffTime);
		}
	}
	
	cbM(cnlAss, curStat, nxtStat);
}
void RL::poll_cbd(void) {
	if ((cbsTme == 0) || (cbsTme > millis())) return;							// timer set to 0 or time for action not reached, leave
	if (cbS) cbS->sendInfoActuatorStatus(cnlAss,getRly(),0);					// call back
	cbsTme = 0;																	// nothing to do any more
}

int RL::getNxtStat() {
  return nxtStat; 
}
int RL::getCurStat() {
  return curStat;
}
void RL::setNxtStat(int newNxtStat) {
  if (nxtStat == newNxtStat) return;
  nxtStat = newNxtStat;
  rlyTime = millis();
}
void RL::setCurStat(int newCurStat) {
  if (curStat == newCurStat) return;
  curStat = newCurStat;
  cbsTme = millis() + ((uint32_t)mDel*1000) + random(((uint32_t)rDel*1000));
  lastTrig = 0;
}

#if defined(USE_SERIAL)
//- -----------------------------------------------------------------------------------------------------------------------
//- serial parser and display functions -----------------------------------------------------------------------------------
//- Parser sketch from: http://jeelabs.org/2010/10/24/parsing-input-commands/
//- -----------------------------------------------------------------------------------------------------------------------
InputParser::InputParser (byte size, Commands *ctab, Stream& stream)
: limit (size), cmds (ctab), io (stream) {
	buffer = (byte*) malloc(size);
	reset();
}
void InputParser::reset() {
	fill = next = 0;
	instring = hexmode = hasvalue = 0;
	top = limit;
}
void InputParser::poll() {
	if (!io.available())
	return;
	char ch = io.read();
	if (ch < ' ' || fill >= top) {
		reset();
		return;
	}
	if (instring) {
		if (ch == '"') {
			buffer[fill++] = 0;
			do
			buffer[--top] = buffer[--fill];
			while (fill > value);
			ch = top;
			instring = 0;
		}
		buffer[fill++] = ch;
		return;
	}
	if (hexmode && (('0' <= ch && ch <= '9') ||
	('A' <= ch && ch <= 'F') ||
	('a' <= ch && ch <= 'f'))) {
		if (!hasvalue)
		value = 0;
		if (ch > '9')
		ch += 9;
		value <<= 4;
		value |= (byte) (ch & 0x0F);
		hasvalue = 1;
		return;
	}
	if ('0' <= ch && ch <= '9') {
		if (!hasvalue)
		value = 0;
		value = 10 * value + (ch - '0');
		hasvalue = 1;
		return;
	}
	hexmode = 0;
	switch (ch) {
		case '$':   hexmode = 1;
		return;
		case '"':   instring = 1;
		value = fill;
		return;
		case ':':   (word&) buffer[fill] = value;
		fill += 2;
		value >>= 16;
		// fall through
		case '.':   (word&) buffer[fill] = value;
		fill += 2;
		hasvalue = 0;
		return;
		case '-':   value = - value;
		hasvalue = 0;
		return;
		case ' ':   if (!hasvalue)
		return;
		// fall through
		case ',':   buffer[fill++] = value;
		hasvalue = 0;
		return;
	}
	if (hasvalue) {
		io.print(F("Unrecognized character: "));
		io.print(ch);
		io.println();
		reset();
		return;
	}
	
	for (Commands* p = cmds; ; ++p) {
		char code = pgm_read_byte(&p->code);
		if (code == 0)
		break;
		if (ch == code) {
			byte bytes = pgm_read_byte(&p->bytes);
			if (fill < bytes) {
				io.print(F("Not enough data, need "));
				io.print((int) bytes);
				io.println(F(" bytes"));
				} else {
				memset(buffer + fill, 0, top - fill);
				((void (*)()) pgm_read_word(&p->fun))();
			}
			reset();
			return;
		}
	}
	
	io.print(F("Known commands:"));
	for (Commands* p = cmds; ; ++p) {
		char code = pgm_read_byte(&p->code);
		if (code == 0)
		break;
		io.print(' ');
		io.print(code);
	}
	io.println();
}
InputParser& InputParser::get(void *ptr, byte len) {
	memcpy(ptr, buffer + next, len);
	next += len;
	return *this;
}
InputParser& InputParser::operator >> (const char*& v) {
	byte offset = buffer[next++];
	v = top <= offset && offset < limit ? (char*) buffer + offset : "";
	return *this;
}
void showPGMText(PGM_P s) {
	char c; 
	while (( c = pgm_read_byte(s++)) != 0) Serial << c;
	Serial << '\n';
}
#endif


//- -----------------------------------------------------------------------------------------------------------------------
//- additional helpers ----------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
extern uint16_t __bss_end, _pHeap_start;
extern void *__brkval;
uint16_t freeMemory() {															// shows free memory
	uint16_t free_memory;

	if((uint16_t)__brkval == 0)
	free_memory = ((uint16_t)&free_memory) - ((uint16_t)&__bss_end);
	else
	free_memory = ((uint16_t)&free_memory) - ((uint16_t)__brkval);

	return free_memory;
}
uint32_t byteTimeCvt(uint8_t tTime) {
	const uint16_t c[8] = {1,10,50,100,600,3000,6000,36000};
	return (uint32_t)(tTime & 0x1f)*c[tTime >> 5]*100;
}
uint32_t intTimeCvt(uint16_t iTime) {
	if (iTime == 0) return 0;
	
	uint8_t tByte;
	if ((iTime & 0x1F) != 0) {
		tByte = 2;
		for (uint8_t i = 1; i < (iTime & 0x1F); i++) tByte *= 2;		
	} else tByte = 1;
	
	return (uint32_t)tByte*(iTime>>5)*100;
}

#if defined(USE_SERIAL)
//- serial print functions 
char pHex(const uint8_t val) {
	const char hexDigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	Serial << hexDigits[val >> 4] << hexDigits[val & 0xF];
	return 0;
}
char pHex(const uint8_t *buf, uint8_t len) {
	for (uint8_t i=0; i<len; i++) {
		pHex(buf[i]);
		if(i+1 < len) Serial << ' ';
	}
	return 0;
}
char pHexL(const uint8_t *buf, uint8_t len) {
	pHex(buf,len);
	Serial << F(" (l:") << len << F(")");
	return 0;
}
char pTime(void) {
	Serial << F("(") << millis() << F(")\n");
	return 0;
}
#endif

//- interrupt handling
void pcInt(uint8_t iPort) {
	cli();																		// all interrupts off
	
	// getting the PCMASK for filtering by interrupt mask
	uint8_t pcMskByte;
	if (iPort == 0) pcMskByte = PCMSK0;
	else if (iPort == 1) pcMskByte = PCMSK1;
	else if (iPort == 2) pcMskByte = PCMSK2;
	else if (iPort == 3) pcMskByte = PCMSK3;
        else {
          sei();
          return; 
        }

	// find the changed pin by getting the pin states for the indicated port, comparing with the stored byte of the port and setting the port byte for the next try
	uint8_t cur = *pci.pAddr[iPort] & pcMskByte;								// get the input byte
	uint8_t msk = pci.lPort[iPort]^cur;											// mask out the changes
	if (!msk) { sei(); return; }												// end while nothing had changed
	
	//Serial << "cur:" << cur << ", lst:" << pci.lPort[iPort] << ", msk:" << msk << ", mbt:" << pcMskByte << '\n';
	pci.lPort[iPort] = cur;														// store the latest port reading

	// finding the respective inxtStatnce of BK by searching for the changed bit
	uint16_t tFnd = (iPort << 8) + msk;											// construct search mask
	for (uint8_t i = 0; i < maxInt; i++) {
		if (tFnd == pci.idx[i]) {												// found; write flag and time in the respective button key class
			//Serial << i << ", cs:" << ((cur & msk)?1:0) << '\n';
			BK *p = pci.ptr[i];
			p->cStat = (cur & msk)?1:0;											// setting the pin status
			p->cTime = millis() + 50;											// for debouncing
			p->cFlag = 1;														// something to do
			break;																// no need to step through all inxtStatnces
		}
	}
	sei();																		// interrupts on again
}
ISR( WDT_vect ) {
	wd_flag = 1;
}
/*
ISR(PCINT0_vect) {
	pcInt(0);
}*/
ISR(PCINT1_vect) {
	pcInt(1);
}
ISR(PCINT2_vect) {
	pcInt(2);
}
ISR(PCINT3_vect) {
	pcInt(3);
}
