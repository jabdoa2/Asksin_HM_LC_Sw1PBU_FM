//- Software config --------------------------------------------------------------------------------------------------------
//#define CC_DBG;															// debug messages of the CC module, ~0.2k program space
//#define SM_DBG;															// debug messages of the SM module, ~1k program space
#define AS_DBG;																// debug messages of the HM module, ~0.6k program space
//#define AS_DBG_Explain;													// debug messages of the HM module, ~5k program space

//- settings of HM device for HM class -------------------------------------------------------------------------------------
const uint8_t devParam[] PROGMEM = {
	/* Firmware version 1 byte */  0x11,									// don't know for what it is good for
	/* Model ID	        2 byte */  0x00, 0xA9,								// model ID, describes HM hardware. we should use high values due to HM starts from 0
	/* Serial ID       10 byte */  'P','S','0','0','0','0','0','0','0','1', // serial ID, needed for pairing
	/* Sub Type ID      1 byte */  0x40,									// not needed for FHEM, it's something like a group ID
	/* Device Info      3 byte */  0x06, 0x00, 0x00							// describes device, not completely clear yet. includes amount of channels
};

static uint8_t  HMID[3]     = { 0x5F, 0xB7, 0x4A };							// very important, must be unique. identifier for the device in the network
static uint8_t  maxRetries  = 3;											// how often a string should be send out until we get an answer
static uint16_t timeOut     = 700;											// time out for ACK handling


//- -----------------------------------------------------------------------------------------------------------------------
//- channel slice definition ----------------------------------------------------------------------------------------------
struct s_chDefType{
	unsigned char  type;
	unsigned short phyAddr;
	unsigned short sliceIdx;
};
struct {
	unsigned char  nbrChannels;
	s_chDefType chDefType[7];
	} const devDef = {
	7                               // number of channels
	,{
		{1,0,0}                // chn:0 type:regDev
		,{0,6,3}                // chn:1 type:regChan
		,{0,15,11}              // chn:2 type:regChan
		,{0,24,19}              // chn:3 type:regChan
		,{0,33,27}              // chn:4 type:regChan
		,{0,42,35}              // chn:5 type:regChan
		,{0,51,43}              // chn:6 type:regChan
	}
};
struct s_sliceStrTpl {
	unsigned char regAddr;
	unsigned char nbrBytes;
	unsigned short phyAddr;
};


// regAddr,nbrBytes,phyAddr
const s_sliceStrTpl sliceStr[] = {
	{0x01, 0x02, 0x0000},           // chn:0 lst:0
	{0x0a, 0x03, 0x0002},
	{0x18, 0x01, 0x0005},
	{0x04, 0x01, 0x0006},           // chn:1 lst:1
	{0x08, 0x02, 0x0007},
	{0x01, 0x01, 0x0009},           // chn:1 lst:4
	{0x01, 0x01, 0x000a},
	{0x01, 0x01, 0x000b},
	{0x01, 0x01, 0x000c},
	{0x01, 0x01, 0x000d},
	{0x01, 0x01, 0x000e},
	{0x04, 0x01, 0x000f},           // chn:2 lst:1
	{0x08, 0x02, 0x0010},
	{0x01, 0x01, 0x0012},           // chn:2 lst:4
	{0x01, 0x01, 0x0013},
	{0x01, 0x01, 0x0014},
	{0x01, 0x01, 0x0015},
	{0x01, 0x01, 0x0016},
	{0x01, 0x01, 0x0017},
	{0x04, 0x01, 0x0018},           // chn:3 lst:1
	{0x08, 0x02, 0x0019},
	{0x01, 0x01, 0x001b},           // chn:3 lst:4
	{0x01, 0x01, 0x001c},
	{0x01, 0x01, 0x001d},
	{0x01, 0x01, 0x001e},
	{0x01, 0x01, 0x001f},
	{0x01, 0x01, 0x0020},
	{0x04, 0x01, 0x0021},           // chn:4 lst:1
	{0x08, 0x02, 0x0022},
	{0x01, 0x01, 0x0024},           // chn:4 lst:4
	{0x01, 0x01, 0x0025},
	{0x01, 0x01, 0x0026},
	{0x01, 0x01, 0x0027},
	{0x01, 0x01, 0x0028},
	{0x01, 0x01, 0x0029},
	{0x04, 0x01, 0x002a},           // chn:5 lst:1
	{0x08, 0x02, 0x002b},
	{0x01, 0x01, 0x002d},           // chn:5 lst:4
	{0x01, 0x01, 0x002e},
	{0x01, 0x01, 0x002f},
	{0x01, 0x01, 0x0030},
	{0x01, 0x01, 0x0031},
	{0x01, 0x01, 0x0032},
	{0x04, 0x01, 0x0033},           // chn:6 lst:1
	{0x08, 0x02, 0x0034},
	{0x01, 0x01, 0x0036},           // chn:6 lst:4
	{0x01, 0x01, 0x0037},
	{0x01, 0x01, 0x0038},
	{0x01, 0x01, 0x0039},
	{0x01, 0x01, 0x003a},
	{0x01, 0x01, 0x003b},
};
struct s_listTpl {
	unsigned char ListNo;
	unsigned char nbrOfSlice;
	unsigned char nbrPeers;
};
struct {
	unsigned char nbrLists;         // number of lists for this channel
	struct s_listTpl type[2];       // fill data with lists
	} const listTypeDef[2] = {
	{ 2                             // type regChan
		,{
			{1,2,1}
			,{4,1,6}
		}
	}
	,{ 1                            // type regDev
		,{
			{0,3,1}
			,{0,0,0}
		}
	}
};

//- -----------------------------------------------------------------------------------------------------------------------
// - peer db config -------------------------------------------------------------------------------------------------------
#define maxChannel 6
#define maxPeer    6
static uint32_t peerdb[maxChannel][maxPeer];
const uint8_t peermax[] = {6,6,6,6,6,6};

//- -----------------------------------------------------------------------------------------------------------------------
// - Channel device config ------------------------------------------------------------------------------------------------
struct s_peer_regChan {                 // chn:6, lst:4
	uint8_t peerNeedsBurst      :1; // reg:0x01, sReg:1
	uint8_t                     :6;
	uint8_t expectAES           :1; // reg:0x01, sReg:1.7
};
struct s_dev_regChan {
	uint8_t                     :4;
	uint8_t longPress           :4; // reg:0x04, sReg:4.4
	uint8_t sign                :1; // reg:0x08, sReg:8
	uint8_t                     :7;
	uint8_t dblPress            :4; // reg:0x09, sReg:9
};
struct s_regChan {
	s_dev_regChan  list1;
	s_peer_regChan peer[6];
};
struct s_regDev {
	uint8_t burstRx;                // reg:0x01, sReg:1
	uint8_t                     :7;
	uint8_t intKeyVisib         :1; // reg:0x02, sReg:2.7
	uint8_t pairCentral[3];         // reg:0x0A, sReg:10
	uint8_t localResDis;            // reg:0x18, sReg:24
};

struct s_regs {
	s_regDev ch_0;
	s_regChan ch_1;
	s_regChan ch_2;
	s_regChan ch_3;
	s_regChan ch_4;
	s_regChan ch_5;
	s_regChan ch_6;
};

struct s_EEPROM {
	unsigned short magNbr;
	uint32_t peerdb[maxChannel][maxPeer];
	s_regs regs;
};


//- -----------------------------------------------------------------------------------------------------------------------
//- struct to provide register settings to user sketch --------------------------------------------------------------------
struct s_cpy_regChan {
	s_dev_regChan  l1;
	s_peer_regChan l4;
};

struct s_regCpy {
	s_regDev    ch0;
	s_cpy_regChan ch1;
	s_cpy_regChan ch2;
	s_cpy_regChan ch3;
	s_cpy_regChan ch4;
	s_cpy_regChan ch5;
	s_cpy_regChan ch6;
} static regMC;

static uint16_t regMcPtr[] = {
	(uint16_t)&regMC.ch0,
	(uint16_t)&regMC.ch1.l1,
	(uint16_t)&regMC.ch1.l4,
	(uint16_t)&regMC.ch2.l1,
	(uint16_t)&regMC.ch2.l4,
	(uint16_t)&regMC.ch3.l1,
	(uint16_t)&regMC.ch3.l4,
	(uint16_t)&regMC.ch4.l1,
	(uint16_t)&regMC.ch4.l4,
	(uint16_t)&regMC.ch5.l1,
	(uint16_t)&regMC.ch5.l4,
	(uint16_t)&regMC.ch6.l1,
	(uint16_t)&regMC.ch6.l4,
};

//- -----------------------------------------------------------------------------------------------------------------------
//- Device definition -----------------------------------------------------------------------------------------------------
//        Channels:      7
//        highest List:  4
//        possible peers:6
//- Memory usage
//        Slices:51
//        EEPROM size:230 fits internal
//        const size: 258


//- -----------------------------------------------------------------------------------------------------------------------
//- main settings to be written very first time to eeprom -----------------------------------------------------------------
//  if 'firstLoad' is defined, hm.init function will step in mainSettings function;
//  be careful, whole eeprom block will be overwritten. you will loose your former settings...
//- -----------------------------------------------------------------------------------------------------------------------
//#define firstLoad;
static void mainSettings(uint16_t *regPtr, uint16_t *peerPtr) {
	static s_regs reg;
	*regPtr = (uint16_t)&reg;
	*peerPtr = (uint16_t)&peerdb;

	reg.ch_0.intKeyVisib = 0;
	reg.ch_0.pairCentral[0] = 0x63;
	reg.ch_0.pairCentral[1] = 0x19;
	reg.ch_0.pairCentral[2] = 0x63;

	
	reg.ch_1.list1.dblPress = 2;
	reg.ch_1.list1.sign = 1;
	reg.ch_1.list1.longPress = 4;
	
	reg.ch_1.peer[0].peerNeedsBurst = 1;
	reg.ch_1.peer[0].expectAES = 1;


	peerdb[0][0] = 0x01086622;
	peerdb[1][0] = 0x02086622;
	
}





