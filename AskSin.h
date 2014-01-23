//- -----------------------------------------------------------------------------------------------------------------------
// AskSin driver implementation
// 2013-08-03 <horst@diebittners.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// Trx868 documentation https://github.com/ccier/openhm/wiki/Trx868
// Parser sketch from: http://jeelabs.org/2010/10/24/parsing-input-commands/
//- -----------------------------------------------------------------------------------------------------------------------#ifndef _ASKSINpH
#define _ASKSINpH

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "Register.h"

// - general struct and declarations ---------------------------------------------------------------------------------------
struct s_jumptable {
	uint8_t code;																// one byte command code
	uint8_t spec;																// one byte command specifier
	void (*fun)(uint8_t, uint8_t*, uint8_t);									// code to call for this command
};

static uint8_t broadCast[] = {0,0,0,0};											// default broadcast address
extern volatile unsigned long timer0_millis;									// make millis timer available for correcting after deep sleep

//- -----------------------------------------------------------------------------------------------------------------------
//- CC1101 communication functions ----------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
class CC {
	public://--------------------------------------------------------------------------------------------------------------

	struct s_trx868 {															// TRX868 communication variables
		uint8_t rfState;														// RF state
		uint8_t crc_ok;															// CRC OK for received message
		uint8_t rssi;															// signal strength
		uint8_t lqi;															// link quality
	}  trx868;

	// TRX868 communication functions
	void    init(void);															// initialize CC1101
	boolean sendData(uint8_t *buf, uint8_t burst);								// send data packet via RF
	uint8_t receiveData(uint8_t *buf);											// read data packet from RX FIFO
	uint8_t detectBurst(void);													// detect burst signal, sleep while no signal, otherwise stay awake
	void    setPowerDownxtStatte(void);											// put CC1101 into power-down state
	uint8_t monitorStatus(void);

	//private://-------------------------------------------------------------------------------------------------------------
	// Hardware definition
	#define PORT_SPI_MISO            PINB
	#define BIT_SPI_MISO             6
	#define PORT_SPI_SS              PORTB
	#define BIT_SPI_SS               4
//	#define GDO0                     2

	#define CC1101_DATA_LEN			 60

	// some register definitions for TRX868 communication
	#define READ_SINGLE              0x80
	#define READ_BURST               0xC0
	#define WRITE_BURST              0x40										// type of transfers
		
	#define CC1101_CONFIG            0x80										// type of register
	#define CC1101_STATUS            0xC0
		
	#define CC1101_PATABLE           0x3E										// PATABLE address
	#define CC1101_TXFIFO            0x3F										// TX FIFO address
	#define CC1101_RXFIFO            0x3F										// RX FIFO address

	#define CC1101_SRES              0x30										// reset CC1101 chip
	#define CC1101_SFSTXON           0x31										// enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). if in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
	#define CC1101_SXOFF             0x32										// turn off crystal oscillator
	#define CC1101_SCAL              0x33										// calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
	#define CC1101_SRX               0x34										// enable RX. perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
	#define CC1101_STX               0x35										// in IDLE state: enable TX. perform calibration first if MCSM0.FS_AUTOCAL=1. if in RX state and CCA is enabled: only go to TX if channel is clear
	#define CC1101_SIDLE             0x36										// exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
	#define CC1101_SWOR              0x38										// start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0
	#define CC1101_SPWD              0x39										// enter power down mode when CSn goes high
	#define CC1101_SFRX              0x3A										// flush the RX FIFO buffer. only issue SFRX in IDLE or RXFIFO_OVERFLOW states
	#define CC1101_SFTX              0x3B										// flush the TX FIFO buffer. only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
	#define CC1101_SWORRST           0x3C										// reset real time clock to Event1 value
	#define CC1101_SNOP              0x3D										// no operation. may be used to get access to the chip status byte

	#define CC1101_PARTNUM           0x30										// status register, chip ID
	#define CC1101_VERSION           0x31										// chip ID
	#define CC1101_FREQEST           0x32										// frequency offset estimate from demodulator
	#define CC1101_LQI               0x33										// demodulator estimate for Link Quality
	#define CC1101_RSSI              0x34										// received signal strength indication
	#define CC1101_MARcurStatTE         0x35										// main radio control state machine state
	#define CC1101_WORTIME1          0x36										// high byte of WOR Time
	#define CC1101_WORTIME0          0x37										// low byte of WOR Time
	#define CC1101_PKTSTATUS         0x38										// current GDOx status and packet status
	#define CC1101_VCO_VC_DAC        0x39										// current setting from PLL calibration module
	#define CC1101_TXBYTES           0x3A										// underflow and number of bytes
	#define CC1101_RXBYTES           0x3B										// overflow and number of bytes
	#define CC1101_RCCTRL1_STATUS    0x3C										// last RC oscillator calibration result
	#define CC1101_RCCTRL0_STATUS    0x3D										// last RC oscillator calibration result

	#define MARcurStatTE_SLEEP          0x00
	#define MARcurStatTE_IDLE           0x01
	#define MARcurStatTE_XOFF           0x02
	#define MARcurStatTE_VCOON_MC       0x03
	#define MARcurStatTE_REGON_MC       0x04
	#define MARcurStatTE_MANCAL         0x05
	#define MARcurStatTE_VCOON          0x06
	#define MARcurStatTE_REGON          0x07
	#define MARcurStatTE_STARTCAL       0x08
	#define MARcurStatTE_BWBOOST        0x09
	#define MARcurStatTE_FS_LOCK        0x0A
	#define MARcurStatTE_IFADCON        0x0B
	#define MARcurStatTE_ENDCAL         0x0C
	#define MARcurStatTE_RX             0x0D
	#define MARcurStatTE_RX_END         0x0E
	#define MARcurStatTE_RX_RST         0x0F
	#define MARcurStatTE_TXRX_SWITCH    0x10
	#define MARcurStatTE_RXFIFO_OFLOW   0x11
	#define MARcurStatTE_FSTXON         0x12
	#define MARcurStatTE_TX             0x13
	#define MARcurStatTE_TX_END         0x14
	#define MARcurStatTE_RXTX_SWITCH    0x15
	#define MARcurStatTE_TXFIFO_UFLOW   0x16


	#define PA_LowPower              0x03										// PATABLE values
	#define PA_Normal                0x50										// PATABLE values
	#define PA_MaxPower			     0xC0
	
	// some macros for TRX868 communication
	#define wait_Miso()       while(bitRead(PORT_SPI_MISO, BIT_SPI_MISO))		// wait until SPI MISO line goes low
	#define cc1101_Select()   bitClear(PORT_SPI_SS, BIT_SPI_SS)					// select (SPI) CC1101
	#define cc1101_Deselect() bitSet(PORT_SPI_SS, BIT_SPI_SS)					// deselect (SPI) CC1101

	// TRX868 communication functions
	uint8_t sendSPI(uint8_t val);												// send bytes to SPI interface
	void    cmdStrobe(uint8_t cmd);												// send command strobe to the CC1101 IC via SPI
	void    readBurst(uint8_t * buf, uint8_t regAddr, uint8_t len);				// read burst data from CC1101 via SPI
	void    writeBurst(uint8_t regAddr, uint8_t* buf, uint8_t len);				// write multiple registers into the CC1101 IC via SPI
	uint8_t readReg(uint8_t regAddr, uint8_t regType);							// read CC1101 register via SPI
	void    writeReg(uint8_t regAddr, uint8_t val);								// write single register into the CC1101 IC via SPI
};

//- -----------------------------------------------------------------------------------------------------------------------
//- status led functions --------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
#define slowRate 600
#define fastRate 300
const uint16_t heartBeat[] = { 50, 200, 50, 1000};
class LD {
	public://--------------------------------------------------------------------------------------------------------------
	void config(uint8_t pin);
	void poll(void);
	void set(uint8_t tMode);
	void stop(void);
	void shortBlink(void);
	void shortBlink3(void);
	
	private://-------------------------------------------------------------------------------------------------------------
	uint8_t  pin;

	uint8_t  state :1;
	uint8_t  mode  :4;
	uint8_t  bCnt  :3;
	uint32_t nTime;

	void on(void);
	void off(void);
	void toggle(void);
};

//- -----------------------------------------------------------------------------------------------------------------------
//- AskSin protocol functions ---------------------------------------------------------------------------------------------
//- with a lot of support from martin876 at FHEM forum
//- -----------------------------------------------------------------------------------------------------------------------
class HM {
	public://--------------------------------------------------------------------------------------------------------------
	// public variables for send and receive
	CC cc;																		// OK, init of the RF/TX module
	LD ld;
	
	#define send_payLoad		(send.data + 10)								// payload for send queue
	#define recv_payLoad		(recv.data + 10)								// payload for receive queue

	struct s_send {																// send queue structure
		uint8_t  data[60];														// buffer for send string
		uint8_t  mCnt;															// message counter
		uint8_t  counter;														// send try counter, has to be 0 if send queue is empty
		uint8_t  retries;														// set max. retries, if message requires an ACK, retries will set to 3
		uint32_t timer;															// timer variable used for store next check time
		uint8_t  burst;															// receiver needs burst signal
	} send;
	struct s_recv {																// receive queue structure
		uint8_t data[60];														// buffer for received string
		uint8_t p_data[60];														// previous buffer, needed while checking against repeated messages
		uint8_t forUs;															// for us indication flag, is set while the received message was addressed to us
		uint8_t  bCast;															// broadcast indication flag
	} recv;
	struct s_powr {
		uint8_t mode;															// indicate the power mode, TX enabled in all modes; 0 = RX enabled,  1 = RX in burst mode, 2 = RX off
		uint8_t state;															// current state of TRX868 module
		uint16_t parTO;															// timeout for pairing in ms
		uint16_t minTO;															// minimum time out in ms
		uint16_t wdTme;															// clock cycle of watch dog in ms
		uint32_t nxtTO;															// check millis() timer against, if millis() >= nextTimeout go in powerdown
	} powr;
	
	s_jumptable *jTblPtr;														// jump table pointer for event handling
	
	// general functions for initializing and operating of module
	HM(s_jumptable *jtPtr, void *mcPtr);										// OK, main object, take over the pointer to jump table for event signalization
	void init(void);															// OK, init function for HM module
	void poll(void);															// OK, main task to manage TX and RX messages
	void send_out(void);														// OK, send function
	void reset(void);															// OK, clear peer database and register content, do a reset of the device
	void setConfigEvent(void);													// OK, raise a config had changed event for main sketch

	void setPowerMode(uint8_t mode);											// set power mode for HM device
	void stayAwake(uint32_t xMillis);											// switch TRX module in RX mode for x milliseconds

	// external functions for pairing and communicating with the module
	void startPairing(void);													// OK, start pairing with master
	void sendInfoActuatorStatus(uint8_t cnl, uint8_t status, uint8_t flag);		// OK, send status function
	void sendACKStatus(uint8_t cnl, uint8_t status, uint8_t douolo);			// OK, send ACK with status
	void sendPeerREMOTE(uint8_t button, uint8_t longPress, uint8_t lowBat);		// (0x40) send REMOTE event to all peers
	void sendPeerRAW(uint8_t cnl, uint8_t type, uint8_t *data, uint8_t len);	// send event to all peers listed in the peers database by channel, type specifies the type of the message, data and len delivers the content of the event
	void send_ACK(void);														// OK, ACK sending function
	void send_NACK(void);														// OK, NACK sending function
	void sendSensorData(uint32_t energyCounter, uint32_t power, uint16_t current, uint16_t voltage, uint8_t frequency);

	// some debug functions
	void printSettings(void);
	void printConfig(void);
	public:	
//	protected://-----------------------------------------------------------------------------------------------------------
	// hardware definition for interrupt handling
	#define GDO0	            2
	#define PORT_GDO0           PIND
//	#define BIT_GDO0            2
	#define INT_GDO0            0

	#define enableIRQ_GDO0()    attachInterrupt(INT_GDO0, isrGDO0event, FALLING);
	#define disableIRQ_GDO0()   detachInterrupt(INT_GDO0);
	static void isrGDO0event(void);												// interrupt to put the bytes from cc1011 to hm
	
	// structure for handling configuration requests
	struct s_conf {
		uint8_t mCnt;															// message counter
		uint8_t reID[3];														// requester id
		uint8_t channel;														// requested channel
		uint8_t list;															// requested list
		uint8_t peer[4];														// requested peer id and peer channel
		uint8_t type;															// message type for answer
		uint8_t wrEn;															// write enabled
		uint8_t act;															// active, 1 = yes, 0 = no
	} conf;

	// structure for handling send peer events
	struct s_pevt {
		uint8_t cnl;															// peer database channel
		uint8_t type;															// message type
		uint8_t data[20];														// data to send
		uint8_t len;															// len of data to send
		uint8_t idx;															// current idx in peer database
		uint8_t sta;															// status message send; 0 no, 1 yes
		uint8_t mFlg;															// message flag
		uint8_t mCnt[maxChannel];												// message counter per channel
		uint8_t eol;															// end of long indicator
		uint8_t act;															// active, 1 = yes, 0 = no
	} pevt;

	// some short hands for receive string handling
	#define recv_len			recv.data[0]									// length of received bytes
	#define recv_rCnt			recv.data[1]									// requester message counter, important for ACK
	#define recv_reID			recv.data+4										// requester ID - who had send the message
	#define recv_msgTp			recv.data[3]									// message type
	#define recv_by10			recv.data[10]									// byte 10 type
	#define recv_by11			recv.data[11]									// byte 11 type

	#define recv_isMsg			bitRead(recv.data[2],7)							// is message, true if 0x80 was set
	#define recv_isRpt			bitRead(recv.data[2],6)							// is a repeated message, true if 0x40 was set
	#define recv_ackRq			bitRead(recv.data[2],5)							// ACK requested, true if 0x20 was set
	#define recv_isCfg			bitRead(recv.data[2],2)							// configuration message, true if 0x04 was set
	#define recv_toMst			bitRead(recv.data[2],1)							// message to master, true if 0x02 was set

	// some polling functions
	void recv_poll(void);														// handles the received string
	void send_poll(void);														// OK, handles the send queue
	void send_conf_poll(void);													// handles information requests
	void send_peer_poll(void);													// handle send events to peers
	void power_poll(void);														// handles the power modes of the TRX868 module

	// receive message handling
	void recv_ConfigPeerAdd(void);												// OK, 01, 01
	void recv_ConfigPeerRemove(void);											// OK, 01, 02
	void recv_ConfigPeerListReq(void);											// OK, 01, 03
	void recv_ConfigParamReq(void);												// OK, 01, 04
	void recv_ConfigStart(void);												// OK, 01, 05
	void recv_ConfigEnd(void);													// OK, 01, 06
	void recv_ConfigWriteIndex(void);											// OK, 01, 08
	void recv_ConfigSerialReq(void);											// OK, 01, 09
	void recv_Pair_Serial(void);												// OK, 01, 0A
	void recv_ConfigStatusReq(void);											// 01, 0E
	void recv_PeerEvent(void);													// OK, 40
	void recv_PairEvent(void);													// 11
	uint8_t recv_Jump(uint8_t tCnl);											// check against jump table to call function in user area, 1 if call was done, 0 if not
	
	// internal send functions
	void send_prep(uint8_t msgCnt, uint8_t comBits, uint8_t msgType, uint8_t *targetID, uint8_t *payLoad, uint8_t payLen);

	// some internal helpers
	void hm_enc(uint8_t *buffer);												// OK, encrypts AskSin payload
	void hm_dec(uint8_t *buffer);												// OK, decrypts AskSin payload
	void exMsg(uint8_t *buf);													// OK, shows enhanced information on the logged communication strings

	
	// - Storage Management -----------------------------------------------------------------------------------------------
	#define magicNumber 1967													// magic number to detect first run
	
	//s_EEPROM *ee;																// pointer to settings in eeprom
	s_regDev regDev;															// structure which holds List0
	s_EEPROM *ee;																// pointer to eeprom structure
	uint16_t mcConfPtr;															// pointer to main channel structure
	uint16_t l3Ptr[maxChannel+1];												// holds pointer for list3 in regMC per channel
	
	// init registers and load default config
	void     initRegisters(void);												// init eeprom and fill registers from eeprom
	
	// slice table functions
	struct s_slcVar { uint8_t slcPtr; uint8_t slcLen; uint16_t phyAddr;	uint8_t phyLen;};
	uint8_t  getSliceDetail(uint8_t cnl, uint8_t lst, uint8_t *peer, s_slcVar *sV);	// OK, returns 1 if ok, 0 on failure
	uint8_t  doesListExist(uint8_t cnl, uint8_t lst);							// OK, check if a list exist, 1 for yes, 0 for no
	uint8_t  getRegList(uint8_t slc_tr, uint8_t slcLen, uint8_t *buf);			// OK, returns len of buffer
	void     getMainChConfig(void);												// OK, load List0 and List1 per channel into a struct
	void     getList3ByPeer(uint8_t cnl, uint8_t *peer);						// loads the device channel into a struct
	// todo: get register address from list for send as actuator
	
	// message generation for TRX868 module
	uint8_t  getListForMsg2(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf);	// OK, create the answer of a info request by filling *buf, returns len of buffer, 0 if done and ff on failure
	uint8_t  getListForMsg3(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf);	// not defined yet
	uint8_t  setListFromMsg(uint8_t cnl, uint8_t lst, uint8_t *peer, const uint8_t *buf, uint8_t len);	// OK, writes the register information in *buf to eeprom, 1 if all went ok, 0 on failure
	uint8_t  getPeerListForMsg(uint8_t cnl, uint8_t *buf);						// OK, create a peer list in the format for answering a peer list request in *buf for the respective channel, returns length of buf, max amount of one junk is 16 bytes, reload the function until len = 0
	
	// peerdb handling; add, remove and search functions
	uint8_t  addPeerFromMsg(uint8_t cnl, uint8_t *peer);						// OK, add a peer to the peer database in the respective channel, returns 0xff on failure, 0 on full and 1 if everything went ok
	uint8_t  removePeerFromMsg(uint8_t cnl, uint8_t *peer);						// OK, remove a 4 byte peer from peer database in the respective channel, returns 0 on not available and 1 if everything is ok
	uint8_t  getCnlByPeer(uint8_t *peer);										// OK, find the respective channel of a 4 byte peer in the peer database, returns the channel 1 to x if the peer is known, 0 if unknown
	uint8_t  getIdxByPeer(uint8_t cnl, uint8_t *peer);							// OK, find the appropriate index of a 4 byte peer in peer database by selecting the channel and searching for the peer, returns peer index, if not found 0xff
	uint8_t  getPeerByIdx(uint8_t cnl, uint8_t idx, uint8_t *peer);				// OK, get the respective 4 byte peer from peer database by selecting channel and index, returns the respective peer in the _tr, function returns 1 if everything is fine, 0 if not
	uint8_t  getFreePeerSlot(uint8_t cnl);										// OK, search for a free slot in peerdb, return the index for a free slot, or 0xff if there is no free slot
	uint8_t  countFreePeerSlot(uint8_t cnl);									// OK, count free slots in peer database by channel
	uint8_t  addPeerToDB(uint8_t cnl, uint8_t *peer);							// OK, add a single peer to database, returns 1 if ok, 0 for failure
        uint8_t  loadDefaultRegset(uint8_t cnl, uint8_t *peer, boolean dual, uint8_t idx);  // Load default regset for new peers

	// to check incoming messages if sender is known
	uint8_t  isPeerKnown(uint8_t *peer);										// OK, check 3 byte peer against peerdb, return 1 if found, otherwise 0
	uint8_t  isPairKnown(uint8_t *pair);										// OK, check 3 byte pair against List0, return 1 if pair is known, otherwise 0
	
	// pure eeprom handling, i2c must implemented
	uint8_t  getEEpromByte(uint16_t addr);										// read EEprom for register config
	void     setEEpromByte(uint16_t addr, uint8_t payload);						// write EEprom for register config
	void     getEEpromBlock(uint16_t addr,uint8_t len,void *ptr);
	void     setEEpromBlock(uint16_t addr,uint8_t len,void *ptr);
};
extern HM hm;

//- -----------------------------------------------------------------------------------------------------------------------
//- button key functions ---------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
class BK {
	public://--------------------------------------------------------------------------------------------------------------
	uint8_t  cFlag :1;															// was there a change happened in key state
	uint8_t  cStat :1;															// current status of the button
	uint32_t cTime;																// stores the next time to check, otherwise we would check every cycle

	void   config(uint8_t Cnl, uint8_t Pin, uint16_t TimeOutShortDbl, uint16_t LongKeyTime, uint16_t TimeOutLongDdbl, void tCallBack(uint8_t, uint8_t));
	void   poll(void);

	private://-------------------------------------------------------------------------------------------------------------
	uint16_t toShDbl;															// minimum time to be recognized as a short key press
	uint16_t lngKeyTme;															// time key should be pressed to be recognized as a long key press
	uint16_t toLoDbl;															// maximum time between a double key press
	void (*callBack)(uint8_t, uint8_t);											// call back address for key display

	uint8_t  pin   :4;															// the pin where the button is connected to
	uint8_t  idx   :4;

	uint8_t  lStat :1;															// last key state
	uint8_t  dblSh :1;															// remember last short key press to indicate a double key press
	uint8_t  dblLo :1;															// remember last long key press to indicate a double key press
	uint8_t  rptLo :1;															// remember long key press to indicate repeated long, or when long was released
	uint32_t kTime;																// stores time when the key was pressed or released

	void   poll_btn(void);														// internal polling function for all inxtStatnces
};

//- -----------------------------------------------------------------------------------------------------------------------
//- relay functions -------------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
class RL {
	public://--------------------------------------------------------------------------------------------------------------
	void    config(uint8_t cnl, void msgCallBack(uint8_t, uint8_t, uint8_t), void adjRlyCallback(uint8_t cnl, uint8_t tValue), HM *statCallBack, uint8_t minDelay, uint8_t randomDelay);

	void    trigger11(uint8_t val, uint8_t *rampTime, uint8_t *duraTime);		// FHEM event
	void    trigger41(uint8_t lngIn, uint8_t val, void *list3);					// sensor event called
	void    trigger40(uint8_t lngIn, uint8_t cnt, void *plist3);				// remote event called
	void    sendStatus(void);													// answer on a status request

	void    poll(void);															// poll handler

        int getNxtStat();
        int getCurStat();
        void setNxtStat(int nxtStat);
        void setCurStat(int curStat);

	private://-------------------------------------------------------------------------------------------------------------
	
	uint8_t curStat:4, nxtStat:4;												// current state and next state
	uint8_t OnDly, OnTime, OffDly, OffTime;										// trigger 40/41 timer variables
	uint8_t lastTrig;															// store for the last trigger
	uint16_t rTime, dTime;														// trigger 11 timer variables
	uint32_t rlyTime;															// timer for poll routine

	uint8_t mDel, rDel;															// store for the call back delay
	void (*cbM)(uint8_t, uint8_t, uint8_t);										// call back address for state change
	HM (*cbS);																	// call back for status message sending
	uint32_t cbsTme;															// timer for call back poll routine	

//	uint8_t hwType:1;															// 0 indicates a monostable, 1 a bistable relay
	uint8_t cnlAss:6;															// remembers channel for the inxtStatnce
//	uint8_t hwPin[4];															// first 2 bytes for on, second two for off

	void    adjRly(uint8_t tValue);												// set the physical status of the relay
	void    (*adjRlyCb)(uint8_t cnl, uint8_t tValue);												// set the physical status of the relay
	uint8_t getRly(void);														// get the status of the relay
	uint8_t getStat(void);														// get the status of the module

	void    poll_rly(void);														// polling function for delay and so on
	void    poll_cbd(void);														// polling function for call back delay
};
#define maxRelay 10
struct s_prl {
	uint8_t nbr;
	RL *ptr[maxRelay];
};

//- -----------------------------------------------------------------------------------------------------------------------
//- serial parser and display functions -----------------------------------------------------------------------------------
//- Parser sketch from: http://jeelabs.org/2010/10/24/parsing-input-commands/
//- -----------------------------------------------------------------------------------------------------------------------
class InputParser {
	public:
	typedef struct {
		char code;															// one-letter command code
		byte bytes;															// number of bytes required as input
		void (*fun)();														// code to call for this command
	} Commands;
	
	InputParser (byte size, Commands*, Stream& =Serial);					// set up with a buffer of specified size
		
	byte count() { return fill; }											// number of data bytes
	byte *buffer;															// holds the read data
	
	void poll();															// call this frequently to check for incoming data
	
	InputParser& operator >> (char& v)      { return get(&v, 1); }
	InputParser& operator >> (byte& v)      { return get(&v, 1); }
	InputParser& operator >> (int& v)       { return get(&v, 2); }
	InputParser& operator >> (word& v)      { return get(&v, 2); }
	InputParser& operator >> (long& v)      { return get(&v, 4); }
	InputParser& operator >> (uint32_t& v)  { return get(&v, 4); }
	InputParser& operator >> (const char*& v);

	private:
	InputParser& get(void*, byte);
	void reset();
	
	byte limit, fill, top, next;
	byte instring, hexmode, hasvalue;
	uint32_t value;
	Commands* cmds;
	Stream& io;
};
extern const InputParser::Commands cmdTab[];
void showPGMText(PGM_P s);

//- -----------------------------------------------------------------------------------------------------------------------
//- additional helpers ----------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
uint16_t freeMemory(void);
uint32_t byteTimeCvt(uint8_t tTime);
uint8_t  int2ByteTimeCvt(uint16_t tTime);
uint32_t intTimeCvt(uint16_t iTime);
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

//- serial print functions
char pHex(const uint8_t val);
char pHex(const uint8_t *buf, uint8_t len);
char pHexL(const uint8_t *buf, uint8_t len);
char pTime(void);

//- interrupt handling
#define maxInt 10
struct s_pci {
	uint8_t	 nbr;
	uint16_t idx[maxInt];
	BK      *ptr[maxInt];
	uint8_t *pAddr[4];
	uint8_t  lPort[4];
};
void pcInt(uint8_t iPort);
static volatile uint8_t wd_flag;
ISR( WDT_vect );
ISR(PCINT0_vect);
ISR(PCINT1_vect);
ISR(PCINT2_vect);
ISR(PCINT3_vect);


