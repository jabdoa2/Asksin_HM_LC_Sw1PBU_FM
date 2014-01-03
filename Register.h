//- Software config --------------------------------------------------------------------------------------------------------
//#define CC_DBG															// debug messages of the CC module, ~0.2k program space
//#define SM_DBG															// debug messages of the SM module, ~1k program space
#define AS_DBG																// debug messages of the HM module, ~0.6k program space
//#define AS_DBG_Explain												// debug messages of the HM module, ~5k program space
#define RL_DBG

//- settings of HM device for HM class -------------------------------------------------------------------------------------
const uint8_t devParam[] PROGMEM = {
	/* Firmware version 1 byte */  0x15,									// don't know for what it is good for
	/* Model ID	        2 byte */  0xF0, 0xA9,	//0x00, 0x6C							// model ID, describes HM hardware. we should use high values due to HM starts from 0
	/* Serial ID       10 byte */  'P','S','0','0','0','0','0','0','0','2', // serial ID, needed for pairing
	/* Sub Type ID      1 byte */  0x10,									// not needed for FHEM, it's something like a group ID
	/* Device Info      3 byte */  0x41, 0x01, 0x00							// describes device, not completely clear yet. includes amount of channels
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
	s_chDefType chDefType[4];
	} const devDef = {
	4                               // number of channels
	,{
		{1,0,0}                // chn:0 type:regDev
		,{2,6,4}                // chn:1 type:regChan_remote
		,{2,15,12}                // chn:2 type:regChan_remote
		,{0,24,20}                // chn:3 type:regChan_actor
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
	{0x08, 0x01, 0x0018},           // chn:3 lst:1
	{0x02, 0x0b, 0x0019},           // chn:3 lst:3
	{0x82, 0x0b, 0x0024},
	{0x02, 0x0b, 0x002f},
	{0x82, 0x0b, 0x003a},
	{0x02, 0x0b, 0x0045},
	{0x82, 0x0b, 0x0050},
	{0x02, 0x0b, 0x005b},
	{0x82, 0x0b, 0x0066},
	{0x02, 0x0b, 0x0071},
	{0x82, 0x0b, 0x007c},
	{0x02, 0x0b, 0x0087},
	{0x82, 0x0b, 0x0092},
};
struct s_listTpl {
	unsigned char ListNo;
	unsigned char nbrOfSlice;
	unsigned char nbrPeers;
};
struct {
	unsigned char nbrLists;         // number of lists for this channel
	struct s_listTpl type[2];       // fill data with lists
	} const listTypeDef[3] = {
	{ 2                             // type regChan actor
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
	,{ 2                             // type regChan remote
		,{
			{1,2,1}
			,{4,1,6}
		}
	}
};

//- -----------------------------------------------------------------------------------------------------------------------
// - peer db config -------------------------------------------------------------------------------------------------------
#define maxChannel 3
#define maxPeer    6
static uint32_t peerdb[maxChannel][maxPeer];
const uint8_t peermax[] = {6,6,6};

//- -----------------------------------------------------------------------------------------------------------------------
// - Channel device config ------------------------------------------------------------------------------------------------
struct s_peer_regChan_actor {                 // chn:1, lst:3
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
struct s_peer_regChan_remote {                 // chn:6, lst:4
	uint8_t peerNeedsBurst      :1; // reg:0x01, sReg:1
	uint8_t                     :6;
	uint8_t expectAES           :1; // reg:0x01, sReg:1.7
};
struct s_dev_regChan_actor {
	uint8_t sign                :1; // reg:0x08, sReg:8
};
struct s_dev_regChan_remote {
	uint8_t                     :4;
	uint8_t longPress           :4; // reg:0x04, sReg:4.4
	uint8_t sign                :1; // reg:0x08, sReg:8
	uint8_t                     :7;
	uint8_t dblPress            :4; // reg:0x09, sReg:9
};
struct s_regChan_actor {
	s_dev_regChan_actor  list1;
	s_peer_regChan_actor peer[6];
};
struct s_regChan_remote {
	s_dev_regChan_remote  list1;
	s_peer_regChan_remote peer[6];
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
	s_regChan_remote ch_1;
	s_regChan_remote ch_2;
	s_regChan_actor ch_3;
};

struct s_EEPROM {
	unsigned short magNbr;
	uint32_t peerdb[maxChannel][maxPeer];
	s_regs regs;
};

//- -----------------------------------------------------------------------------------------------------------------------
//- struct to provide register settings to user sketch --------------------------------------------------------------------

struct s_cpy_regChan_actor {
	s_dev_regChan_actor  l1;
	s_peer_regChan_actor l3;
};

struct s_cpy_regChan_remote {
	s_dev_regChan_remote  l1;
	s_peer_regChan_remote l4;
};

struct s_regCpy {
	s_regDev    ch0;
	s_cpy_regChan_remote ch1;
	s_cpy_regChan_remote ch2;
	s_cpy_regChan_actor ch3;
} static regMC;

static uint16_t regMcPtr[] = {
	(uint16_t)&regMC.ch0,
	(uint16_t)&regMC.ch1.l1,
	(uint16_t)&regMC.ch1.l4,
	(uint16_t)&regMC.ch2.l1,
	(uint16_t)&regMC.ch2.l4,
	(uint16_t)&regMC.ch3.l1,
	(uint16_t)&regMC.ch3.l3,
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
#define firstLoad
static void mainSettings(uint16_t *regPtr, uint16_t *peerPtr) {
	static s_regs reg;
	*regPtr = (uint16_t)&reg;
	*peerPtr = (uint16_t)&peerdb;

	reg.ch_0.intKeyVisib = 0;
	reg.ch_0.pairCentral[0] = 0x1A;
	reg.ch_0.pairCentral[1] = 0xB1;
	reg.ch_0.pairCentral[2] = 0x50;

	
	reg.ch_1.list1.dblPress = 2;
	reg.ch_1.list1.sign = 0;
	reg.ch_1.list1.longPress = 4;
	
	reg.ch_1.peer[0].peerNeedsBurst = 1;
	reg.ch_1.peer[0].expectAES = 0;

	reg.ch_2.peer[0].peerNeedsBurst = 1;

        reg.ch_3.peer[0].shActionType = 1;
        reg.ch_3.peer[0].lgActionType = 1;
        reg.ch_3.peer[0].shSwJtOff = 3;
        reg.ch_3.peer[0].lgSwJtOff = 3;
        reg.ch_3.peer[0].shSwJtOn = 6;
        reg.ch_3.peer[0].lgSwJtOn = 6;

	peerdb[0][0] = 0x013BD621; // 21D63B
	peerdb[1][0] = 0x013BD621;
	peerdb[2][0] = 0x01563412; // 12345601
	
}





