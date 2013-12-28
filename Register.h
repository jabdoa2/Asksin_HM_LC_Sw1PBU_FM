//- Software config --------------------------------------------------------------------------------------------------------
//#define CC_DBG;															// debug messages of the CC module, ~0.2k program space
//#define SM_DBG;															// debug messages of the SM module, ~1k program space
#define AS_DBG;																// debug messages of the HM module, ~0.6k program space
//#define AS_DBG_Explain;													// debug messages of the HM module, ~5k program space
#define RL_DBG;

//- settings of HM device for HM class -------------------------------------------------------------------------------------
const uint8_t devParam[] PROGMEM = {
	/* Firmware version 1 byte */  0x15,									// don't know for what it is good for
	/* Model ID	        2 byte */  0x00, 0x6C,								// model ID, describes HM hardware. we should use high values due to HM starts from 0
	/* Serial ID       10 byte */  'P','S','0','0','0','0','0','0','0','2', // serial ID, needed for pairing
	/* Sub Type ID      1 byte */  0x10,									// not needed for FHEM, it's something like a group ID
	/* Device Info      3 byte */  0x41, 0x01, 0x00							// describes device, not completely clear yet. includes amount of channels
};

static uint8_t  HMID[3]     = { 0x5F, 0xB7, 0x4B };							// very important, must be unique. identifier for the device in the network
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
	s_chDefType chDefType[2];
	} const devDef = {
	2                               // number of channels
	,{
		{1,0,0}                // chn:0 type:regDev
		,{0,6,4}                // chn:1 type:regChan
	}
};
struct s_sliceStrTpl {
	unsigned char regAddr;
	unsigned char nbrBytes;
	unsigned short phyAddr;
};


// regAddr,nbrBytes,phyAddr
const s_sliceStrTpl sliceStr[] = {
	{0x02, 0x01, 0x0000},           // chn:0 lst:0
	{0x05, 0x01, 0x0001},
	{0x0a, 0x03, 0x0002},
	{0x12, 0x01, 0x0005},
	{0x08, 0x01, 0x0006},           // chn:1 lst:1
	{0x02, 0x0b, 0x0007},           // chn:1 lst:3
	{0x82, 0x0b, 0x0012},
	{0x02, 0x0b, 0x001d},
	{0x82, 0x0b, 0x0028},
	{0x02, 0x0b, 0x0033},
	{0x82, 0x0b, 0x003e},
	{0x02, 0x0b, 0x0049},
	{0x82, 0x0b, 0x0054},
	{0x02, 0x0b, 0x005f},
	{0x82, 0x0b, 0x006a},
	{0x02, 0x0b, 0x0075},
	{0x82, 0x0b, 0x0080},
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
			{1,1,1}
			,{3,2,6}
		}
	}
	,{ 1                            // type regDev
		,{
			{0,4,1}
			,{0,0,0}
		}
	}
};

//- -----------------------------------------------------------------------------------------------------------------------
// - peer db config -------------------------------------------------------------------------------------------------------
#define maxChannel 1
#define maxPeer    6
static uint32_t peerdb[maxChannel][maxPeer];
const uint8_t peermax[] = {6};

//- -----------------------------------------------------------------------------------------------------------------------
// - Channel device config ------------------------------------------------------------------------------------------------
struct s_peer_regChan {                 // chn:1, lst:3
	uint8_t shCtDlyOn           :4; // reg:0x02, sReg:2
	uint8_t shCtDlyOff          :4; // reg:0x02, sReg:2.4
	uint8_t shCtOn              :4; // reg:0x03, sReg:3
	uint8_t shCtOff             :4; // reg:0x03, sReg:3.4
	uint8_t shCtValLo;              // reg:0x04, sReg:4
	uint8_t shCtValHi;              // reg:0x05, sReg:5
	uint8_t shOnDly;                // reg:0x06, sReg:6
	uint8_t shOnTime;               // reg:0x07, sReg:7
	uint8_t shOffDly;               // reg:0x08, sReg:8
	uint8_t shOffTime;              // reg:0x09, sReg:9
	uint8_t shActionType        :2; // reg:0x0A, sReg:10
	uint8_t                     :4;
	uint8_t shOffTimeMode       :1; // reg:0x0A, sReg:10.6
	uint8_t shOnTimeMode        :1; // reg:0x0A, sReg:10.7
	uint8_t shSwJtOn            :4; // reg:0x0B, sReg:11
	uint8_t shSwJtOff           :4; // reg:0x0B, sReg:11.4
	uint8_t shSwJtDlyOn         :4; // reg:0x0C, sReg:12
	uint8_t shSwJtDlyOff        :4; // reg:0x0C, sReg:12.4
	uint8_t lgCtDlyOn           :4; // reg:0x82, sReg:130
	uint8_t lgCtDlyOff          :4; // reg:0x82, sReg:130.4
	uint8_t lgCtOn              :4; // reg:0x83, sReg:131
	uint8_t lgCtOff             :4; // reg:0x83, sReg:131.4
	uint8_t lgCtValLo;              // reg:0x84, sReg:132
	uint8_t lgCtValHi;              // reg:0x85, sReg:133
	uint8_t lgOnDly;                // reg:0x86, sReg:134
	uint8_t lgOnTime;               // reg:0x87, sReg:135
	uint8_t lgOffDly;               // reg:0x88, sReg:136
	uint8_t lgOffTime;              // reg:0x89, sReg:137
	uint8_t lgActionType        :2; // reg:0x8A, sReg:138
	uint8_t                     :3;
	uint8_t lgMultiExec         :1; // reg:0x8A, sReg:138.5
	uint8_t lgOffTimeMode       :1; // reg:0x8A, sReg:138.6
	uint8_t lgOnTimeMode        :1; // reg:0x8A, sReg:138.7
	uint8_t lgSwJtOn            :4; // reg:0x8B, sReg:139
	uint8_t lgSwJtOff           :4; // reg:0x8B, sReg:139.4
	uint8_t lgSwJtDlyOn         :4; // reg:0x8C, sReg:140
	uint8_t lgSwJtDlyOff        :4; // reg:0x8C, sReg:140.4
};
struct s_dev_regChan {
	uint8_t sign                :1; // reg:0x08, sReg:8
};
struct s_regChan {
	s_dev_regChan  list1;
	s_peer_regChan peer[6];
};
struct s_regDev {
	uint8_t                     :7;
	uint8_t intKeyVisib         :1; // reg:0x02, sReg:2.7
	uint8_t                     :6;
	uint8_t ledMode             :2; // reg:0x05, sReg:5.6
	uint8_t pairCentral[3];         // reg:0x0A, sReg:10
	uint8_t lowBatLimitBA;          // reg:0x12, sReg:18
};

struct s_regs {
	s_regDev ch_0;
	s_regChan ch_1;
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
	s_peer_regChan l3;
};

struct s_regCpy {
	s_regDev    ch0;
	s_cpy_regChan ch1;
} static regMC;

static uint16_t regMcPtr[] = {
	(uint16_t)&regMC.ch0,
	(uint16_t)&regMC.ch1.l1,
	(uint16_t)&regMC.ch1.l3,
};

//- -----------------------------------------------------------------------------------------------------------------------
//- Device definition -----------------------------------------------------------------------------------------------------
//        Channels:      2
//        highest List:  3
//        possible peers:6
//- Memory usage
//        Slices:17
//        EEPROM size:189 fits internal
//        const size: 92


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


	peerdb[0][0] = 0x01086622;
	peerdb[0][1] = 0x02086622;
	
}





