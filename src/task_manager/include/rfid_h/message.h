#ifndef GREADER_MESSAGE_H
#define GREADER_MESSAGE_H

#define REGION(msg) 1

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    AntennaNo_1 = 0x1,
    AntennaNo_2 = 0x2,
    AntennaNo_3 = 0x4,
    AntennaNo_4 = 0x8,
    AntennaNo_5 = 0x10,
    AntennaNo_6 = 0x20,
    AntennaNo_7 = 0x40,
    AntennaNo_8 = 0x80,
    AntennaNo_9 = 0x100,
    AntennaNo_10 = 0x200,
    AntennaNo_11 = 0x400,
    AntennaNo_12 = 0x800,
    AntennaNo_13 = 0x1000,
    AntennaNo_14 = 0x2000,
    AntennaNo_15 = 0x4000,
    AntennaNo_16 = 0x8000,
    AntennaNo_17 = 0x10000,
    AntennaNo_18 = 0x20000,
    AntennaNo_19 = 0x40000,
    AntennaNo_20 = 0x80000,
    AntennaNo_21 = 0x100000,
    AntennaNo_22 = 0x200000,
    AntennaNo_23 = 0x400000,
    AntennaNo_24 = 0x800000,
    AntennaNo_25 = 0x1000000,
    AntennaNo_26 = 0x2000000,
    AntennaNo_27 = 0x4000000,
    AntennaNo_28 = 0x8000000,
    AntennaNo_29 = 0x10000000,
    AntennaNo_30 = 0x20000000,
    AntennaNo_31 = 0x40000000,
    AntennaNo_32 = 0x80000000,
} AntennaNo;

typedef enum
{
    InitInventory = 0,
    InitUpgrade = 1,
} ConnectionAttemptEventStatusType;

typedef enum
{
    InventoryMode_Single = 0,
    InventoryMode_Inventory = 1,
} InventoryMode;

typedef struct
{
    int RtCode;
    char RtMsg[128];
} Result;

#if REGION("查询读写器信息")
typedef struct
{
    Result rst;
    char SerialNum[64]; //读写器流水号
    long PowerOnTime; //上电时间
    char BaseBuildDate[64]; //基带编译时间
    char AppVersion[16]; //应用软件版本（如：“0.1.0.0”）
    char AppBuildDate[32]; //应用编译时间
    char SystemVersion[64]; //操作系统版本
} MsgAppGetReaderInfo;
#endif

#if REGION("基带版本号")
typedef struct
{
    Result rst;
    char BaseVersion[16]; //基带软件版本
} MsgAppGetBaseVersion;
#endif

#if REGION("串口参数")
typedef struct
{
    Result rst;
    char BaudrateIndex;
} MsgAppSetSerialParam;

typedef struct
{
    Result rst;
    char BaudrateIndex;
} MsgAppGetSerialParam;
#endif

#if REGION("配置读写器以太网IP参数")
typedef struct
{
    Result rst;
    unsigned char autoIp;
    char ip[32];
    char mask[32];
    char gateway[32];
    char dns1[32];
    char dns2[32];
} MsgAppSetEthernetIP;

typedef struct
{
    Result rst;
    unsigned char autoIp;
    char ip[32];
    char mask[32];
    char gateway[32];
    char dns1[32];
    char dns2[32];
} MsgAppGetEthernetIP;
#endif

#if REGION("查询读写器以太网MAC")
typedef struct
{
    Result rst;
    char mac[32];
} MsgAppGetEthernetMac;
#endif

#if REGION("配置和查询服务器/客户端模式参数")
typedef struct
{
    Result rst;
    unsigned char tcpMode;
    unsigned short serverPort;
    char clientIp[32];
    unsigned short clientPort;
} MsgAppSetTcpMode;

typedef struct
{
    Result rst;
    unsigned char tcpMode;
    unsigned short serverPort;
    char clientIp[32];
    unsigned short clientPort;
} MsgAppGetTcpMode;
#endif

#if REGION("设置GPO")
typedef struct
{
    int GpoIndex; //Gpo1-32
    unsigned char Gpo; //0, 继电器断开；1，继电器闭合
} DictionaryGpo;

typedef struct
{
    Result rst;
    DictionaryGpo DicGpo[128];
    unsigned char DicGpoCount;
} MsgAppSetGpo;
#endif

#if REGION("获取GPI状态")
typedef struct
{
    int GpiIndex; //Gpi1-16
    unsigned char Gpi; //0, 低电平；1，高电平
} DictionaryGpi;

typedef struct
{
    Result rst;
    DictionaryGpi DicGpi[128];
    unsigned char DicGpiCount;
} MsgAppGetGpiState;
#endif

#if REGION("GPI触发参数")
typedef struct
{
    Result rst;
    char GpiPort; //GPI 端口号，索引从0 开始
    char TriggerStart; //触发开始（0 触发关闭，1 低电平触发，2 高电平触发，3 上升沿触发，4 下降沿触发，5 任意边沿触发）
    char TriggerCommand[128]; //触发绑定命令（Hex字符串）,由调用者释放
    char TriggerOver; //触发停止（0 不停止，1 低电平触发，2 高电平触发，3 上升沿触发，4 下降沿触发，5 任意边沿触发，6 延时停止）
    unsigned short OverDelayTime; //延时停止时间（仅当停止条件为“延时停止”生效）, 以10ms为单位
    char LevelUploadSwitch; //触发不停止时IO 电平变化上传开关（0 不上传，1 上传）
} MsgAppSetGpiTrigger;

typedef struct
{
    Result rst;
    char GpiPort; //GPI 端口号，索引从0 开始
    char TriggerStart; //触发开始（0 触发关闭，1 低电平触发，2 高电平触发，3 上升沿触发，4 下降沿触发，5 任意边沿触发）
    char TriggerCommand[128]; //触发绑定命令（Hex字符串）
    char TriggerOver; //触发停止（0 不停止，1 低电平触发，2 高电平触发，3 上升沿触发，4 下降沿触发，5 任意边沿触发，6 延时停止）
    unsigned short OverDelayTime; //延时停止时间（仅当停止条件为“延时停止”生效）, 以10ms为单位
    char LevelUploadSwitch; //触发不停止时IO 电平变化上传开关（0 不上传，1 上传）
} MsgAppGetGpiTrigger;
#endif

#if REGION("韦根通信参数")
typedef struct
{
    Result rst;
    char Switch;
    char Format;
    char Content;
    short NegativePulseWidth;
    unsigned short PulseInterval;
} MsgAppSetWiegand;

typedef struct
{
    Result rst;
    char Switch;
    char Format;
    char Content;
    short NegativePulseWidth;
    unsigned short PulseInterval;
} MsgAppGetWiegand;
#endif

#if REGION("重启读写器")
typedef struct
{
} MsgAppReboot;
#endif

#if REGION("读写器系统时间")
typedef struct
{
    Result rst;
    int UtcSecond;
    int UtcMicrosecond;
} MsgAppSetReaderTime;

typedef struct
{
    Result rst;
    int UtcSecond;
    int UtcMicrosecond;
} MsgAppGetReaderTime;
#endif

#if REGION("连接状态确认")
typedef struct
{
    Result rst;
    unsigned char param[5];
} MsgAppHeartbeat;
#endif

#if REGION("读写器MAC")
typedef struct
{
    Result rst;
    unsigned char Mac[6];
} MsgAppSetReaderMac;
#endif

#if REGION("恢复读写器默认配置")
typedef struct
{
    Result rst;
} MsgAppRestoreDefault;
#endif

#if REGION("读写器RS485参数")
typedef struct
{
    Result rst;
    char Address;
    char BaudRate;
} MsgAppSetRs485;

typedef struct
{
    Result rst;
    char Address;
    char BaudRate;
} MsgAppGetRs485;
#endif

#if REGION("读写断点续传功能")
typedef struct
{
    Result rst;
    char Switch;
} MsgAppSetBreakpointResume;

typedef struct
{
    Result rst;
    char Switch;
} MsgAppGetBreakpointResume;
#endif

#if REGION("读写器缓存标签数据")
typedef struct
{
    Result rst;
} MsgAppGetCacheTagData;

typedef struct
{
    Result rst;
} MsgAppClearCacheTagData;

typedef struct
{
    Result rst;
    int SerialNumber;
} MsgAppTagDataReply;
#endif

#if REGION("蜂鸣器")
typedef struct
{
    Result rst;
    char Switch;
} MsgAppSetBuzzerSwitch;

typedef struct
{
    Result rst;
    char Status;
    char Mode;
} MsgAppSetBuzzerCtrl;
#endif

#if REGION("白名单参数")
typedef struct
{
    Result rst;
    int PacketNumber;
    unsigned char PacketContent[512];
    int PacketLen;
} MsgAppGetWhiteList;

typedef struct
{
    Result rst;
    int PacketNumber;
    unsigned char PacketContent[512];
    int PacketLen;
} MsgAppImportWhiteList;

typedef struct
{
    Result rst;
} MsgAppDelWhiteList;

typedef struct
{
    Result rst;
    char RelayNo;
    short RelayCloseTime;
} MsgAppSetWhiteListAction;

typedef struct
{
    Result rst;
    char RelayNo;
    short RelayCloseTime;
} MsgAppGetWhiteListAction;

typedef struct
{
    Result rst;
    char Switch;
    char FilterArea;
} MsgAppSetWhiteListSwitch;

typedef struct
{
    Result rst;
    char Switch;
    char FilterArea;
} MsgAppGetWhiteListSwitch;
#endif

#if REGION("UDP上报参数")
typedef struct
{
    Result rst;
    char Switch;
    char ip[32];
    int port;
    int period;
} MsgAppSetUdpParam;

typedef struct
{
    Result rst;
    char Switch;
    char ip[32];
    int port;
    int period;
} MsgAppGetUdpParam;
#endif

#if REGION("HTTP上报参数")
typedef struct
{
    Result rst;
    char Switch;
    int period;
    char format;
    int timeout;
    char cache;
    int addressLen;
    char address[512];
} MsgAppSetHttpParam;

typedef struct
{
    Result rst;
    char Switch;
    int period;
    char format;
    int timeout;
    char cache;
    int addressLen;
    char address[512];
} MsgAppGetHttpParam;
#endif

#if REGION("USB键盘开关")
typedef struct
{
    Result rst;
    char operate;
    char kbdSwitch;
} MsgAppUsbKbdSwitch;
#endif


#if REGION("WIFI参数")
typedef struct
{
    Result rst;
} MsgAppSetWifiHotspotSearch;

typedef struct
{
    Result rst;
} WifiHotspotInfo;

typedef struct
{
    Result rst;
    int PacketNumber;
    char PacketContent[1024];
} MsgAppGetWifiHotspotSearch;

typedef struct
{
    Result rst;
    int ESSIDLen;
    char ESSID[512];
    int pwdLen;
    char password[512];
    bool setCTFlag;
    char certificationType;
    bool setEAFlag;
    char encryptionAlgorithm;
} MsgAppSetWifiHotspot;

typedef struct
{
    Result rst;
    int ESSIDLen;
    char ESSID[512];
} MsgAppGetWifiConnectStatus;

typedef struct
{
    Result rst;
    char autoIp;
    char ip[32];
    char mask[32];
    char gateway[32];
    char dns1[32];
    char dns2[32];
    int ESSIDLen;
    char ESSID[512];
} MsgAppSetWifiIp;

typedef struct
{
    Result rst;
    int ESSIDLen;
    char ESSID[512];
    char autoIp;
    char ip[32];
    char mask[32];
    char gateway[32];
    char dns1[32];
    char dns2[32];
} MsgAppGetWifiIp;

typedef struct
{
    Result rst;
    char Switch;
} MsgAppSetWifiOnOff;

typedef struct
{
    Result rst;
    char Switch;
} MsgAppGetWifiOnOff;
#endif

#if REGION("EAS匹配报警参数")
typedef struct
{
    short keepTime;
    char setGpo1;
    char gpo1;
    char setGpo2;
    char gpo2;
    char setGpo3;
    char gpo3;
    char setGpo4;
    char gpo4;
} ActionParamSuccess;

typedef struct
{
    short keepTime;
    char setGpo1;
    char gpo1;
    char setGpo2;
    char gpo2;
    char setGpo3;
    char gpo3;
    char setGpo4;
    char gpo4;
} ActionParamFail;

typedef struct
{
    Result rst;
    char alarmSwitch;
    char filterDataArea;
    short start;
    int contentLen;
    char content[512];
    int maskLen;
    char mask[512];
    bool setSuccessFlag;
    ActionParamSuccess success;
    bool setFailFlag;
    ActionParamFail fail;
} MsgAppSetEasAlarm;

typedef struct
{
    Result rst;
    char alarmSwitch;
    char filterData;
    short start;
    int contentLen;
    char content[512];
    int maskLen;
    char mask[512];
    bool setSuccessFlag;
    ActionParamSuccess success;
    bool setFailFlag;
    ActionParamFail fail;
} MsgAppGetEasAlarm;
#endif

#if REGION("WiFi漫游检测开关")
typedef struct
{
    Result rst;
    char Operate;
    char Switch;
} MsgAppWifiRoaming;
#endif

#if REGION("看门狗喂狗开关")
typedef struct
{
    Result rst;
    char Operate;
    char Switch;
} MsgAppWatchDog;
#endif

#if REGION("查询读写器RFID能力")
typedef struct
{
    Result rst;
    unsigned char MaxPower; //最大支持功率
    unsigned char MinPower; //最小支持功率
    unsigned char AntennaCount; //天线数量
    int FrequencyArraySize;
    unsigned char FrequencyArray[16]; //支持的频段列表
    int ProtocolArraySize;
    unsigned char ProtocolArray[16]; //支持的协议列表
} MsgBaseGetCapabilities;
#endif

#if REGION("配置和查询读写器功率")
typedef struct
{
    int AntennaNo;
    unsigned char Power;
} DictionaryPower;

typedef struct
{
    Result rst;
    DictionaryPower DicPower[128];
    int DicPowerCount;
    bool setReadOrWriteFlag;
    char ReadOrWrite;
    bool setPowerDownSaveFlag;
    char PowerDownSave;
} MsgBaseSetPower;

typedef struct
{
    Result rst;
    bool setReadOrWrite;
    char ReadOrWrite;
    DictionaryPower DicPower[128];
    int DicPowerCount;
} MsgBaseGetPower;

#endif

#if REGION("配置和查询读写器RF频段")
typedef struct
{
    Result rst;
    unsigned char FreqRangeIndex;
} MsgBaseSetFreqRange;

typedef struct
{
    Result rst;
    unsigned char FreqRangeIndex;
} MsgBaseGetFreqRange;
#endif

#if REGION("配置和查询读写器工作频率")
typedef struct
{
    Result rst;

    int Mode;

    int FrequenciesSize;
    unsigned char Frequencies[512];

    bool setPowerDownSaveFlag;
    unsigned char PowerDownSave;
} MsgBaseSetFrequency;

typedef struct
{
    Result rst;
    int Mode;
    int FrequenciesSize;
    unsigned char Frequencycies[512];
} MsgBaseGetFrequency;
#endif

#if REGION("配置和查询天线扩展参数")
typedef struct
{
    unsigned char AntennaNo;
    unsigned char ChildAntenna;
} AntDictionary;

typedef struct
{
    Result rst;
    unsigned char DicCount;
    AntDictionary DicAntenna[256];
} MsgBaseSetAntennaHub;

typedef struct
{
    Result rst;
    unsigned char DicCount;
    AntDictionary DicAntenna[256];
} MsgBaseGetAntennaHub;
#endif

#if REGION("配置和查询标签上传参数")
typedef struct
{
    Result rst;

    bool setRepeatedTimeFlag;
    unsigned short RepeatedTime; //  重复标签过滤时间（可选）（表示在一个读卡指令执行周期内，在指定的重复过滤时间内相同的标签内容只上传一次，0~65535，时间单位：10ms）。

    bool setRssiTVFlag;
    unsigned char RssiTV; //RSSI 阈值（可选）（标签RSSI 值低于阈值时标签数据将不上传并丢弃）。
} MsgBaseSetTagLog;

typedef struct
{
    Result rst;

    bool setRepeatedTimeFlag;
    unsigned short RepeatedTime; //  重复标签过滤时间（可选）（表示在一个读卡指令执行周期内，在指定的重复过滤时间内相同的标签内容只上传一次，0~65535，时间单位：10ms）。

    bool setRssiTVFlag;
    unsigned char RssiTV; //RSSI 阈值（可选）（标签RSSI 值低于阈值时标签数据将不上传并丢弃）。
} MsgBaseGetTagLog;
#endif

#if REGION("配置和查询EPC基带参数")
typedef struct
{
    Result rst;

    bool setBaseSpeedFlag;
    unsigned char BaseSpeed; // EPC 基带速率（可选）。

    bool setQValueFlag;
    unsigned char QValue; // 默认Q 值（可选）(0~15)。

    bool setSessionFlag;
    unsigned char Session; // （可选）(0,Session0; 1,Session1; 2,Session2; 3,Session3)。

    bool setInventoryFlag;
    unsigned char InventoryFlag; // 盘存标志参数（可选）(0,仅用Flag A 盘存;1,仅用Flag B 盘存;2,轮流使用Flag	A 和Flag B)。
} MsgBaseSetBaseband;

typedef struct
{
    Result rst;

    bool setBaseSpeedFlag;
    unsigned char BaseSpeed; // EPC 基带速率（可选）。

    bool setQValueFlag;
    unsigned char QValue; // 默认Q 值（可选）(0~15)。

    bool setSessionFlag;
    unsigned char Session; // （可选）(0,Session0; 1,Session1; 2,Session2; 3,Session3)。

    bool setInventoryFlag;
    unsigned char InventoryFlag; // 盘存标志参数（可选）(0,仅用Flag A 盘存;1,仅用Flag B 盘存;2,轮流使用Flag	A 和Flag B)。
} MsgBaseGetBaseband;

#endif

#if REGION("配置和查询自动空闲模式")
typedef struct
{
    Result rst;
    unsigned char OnOff;

    bool setFreeTimeFlag;
    unsigned short FreeTime;
} MsgBaseSetAutoDormancy;

typedef struct
{
    Result rst;
    unsigned char OnOff;

    bool setFreeTimeFlag;
    unsigned short FreeTime;
} MsgBaseGetAutoDormancy;
#endif

#if REGION("配置和查询读写器驻留时间参数")
typedef struct
{
    Result rst;

    bool setAntResidenceTimeFlag;
    unsigned short AntResidenceTime;

    bool setFreqResidenceTimeFlag;
    unsigned short FreqResidenceTime;
} MsgBaseSetResidenceTime;

typedef struct
{
    Result rst;

    bool setAntResidenceTimeFlag;
    unsigned short AntResidenceTime;

    bool setFreqResidenceTimeFlag;
    unsigned short FreqResidenceTime;
} MsgBaseGetResidenceTime;
#endif

#if REGION("配置和查询GB基带参数")
typedef struct
{
    char Tc;
    char Trext;
    char rlf;
    char rlc;
} ParamBaseSpeed;

typedef struct
{
    char CIN;
    char CCN;
} ParamAntiCollision;

typedef struct
{
    Result rst;

    bool setGbBaseSpeedFlag;
    ParamBaseSpeed GbBaseSpeed;

    bool setGbAntiCollisionFlag;
    ParamAntiCollision GbAntiCollision;

    bool setSessionFlag;
    unsigned char Session;

    bool setInventoryFlag;
    unsigned char InventoryFlag;
} MsgBaseSetGbBaseband;

typedef struct
{
    Result rst;

    ParamBaseSpeed GbBaseSpeed;

    ParamAntiCollision GbAntiCollision;

    unsigned char Session;

    unsigned char InventoryFlag;
} MsgBaseGetGbBaseband;
#endif

#if REGION("配置和查询GJB基带参数")
typedef struct
{
    Result rst;

    bool setGjbBaseSpeedFlag;
    ParamBaseSpeed GjbBaseSpeed;

    bool setGjbAntiCollisionFlag;
    ParamAntiCollision GjbAntiCollision;

    bool setInventoryFlag;
    unsigned char InventoryFlag;
} MsgBaseSetGjbBaseband;

typedef struct
{
    Result rst;

    ParamBaseSpeed GjbBaseSpeed;

    ParamAntiCollision GjbAntiCollision;

    unsigned char InventoryFlag;
} MsgBaseGetGjbBaseband;
#endif

#if REGION("读6c")
typedef struct
{
    Result rst;
    char Area;
    unsigned short BitStart;
    int BitLen; //位长度
    //char* HexData; //需要匹配的数据内容,十六进制
    char HexData[2048];
} ParamFilter;

typedef struct
{
    Result rst;
    char Mode;
    unsigned char Len;
} ParamReadTid;

typedef struct
{
    Result rst;
    unsigned short Start;
    unsigned char Len;
} ParamReadUserdata;

typedef struct
{
    Result rst;
    unsigned short Start;
    unsigned char Len;
} ParamReadReserved;

typedef struct
{
    Result rst;
    unsigned short Start;
    unsigned char Len;
} ParamReadEpcBank;

typedef struct
{
    Result rst;
    unsigned char Fastid;
    unsigned char Foucs;
} ImpinjMonza;

typedef struct
{
    Result rst;
    long Index;
    ParamFilter Filter;
} MsgBaseSetMultiFilter;

typedef struct
{
    Result rst;
    long Index;
    ParamFilter Filter;
} MsgBaseGetMultiFilter;

typedef struct
{
    Result rst;
    int AntennaEnable;
    char InventoryMode;
    ParamFilter Filter;
    ParamReadTid ReadTid;
    ParamReadUserdata ReadUserdata;
    ParamReadReserved ReadReserved;
    char StrHexPassword[9];
    int MonzaQtPeek;
    int Rfmicron;
    int EMSensorData;
    ParamReadEpcBank EpcBank;
    ImpinjMonza monza;
    int Ctesius;
    int Quanray;
    int Timeout;
} MsgBaseInventoryEpc;
#endif

#if REGION("写EPC")
typedef struct
{
    Result rst;
    int AntennaEnable; // 天线端口（使用天线枚举，详见AntennaNo,多个天线使用或）
    char Area; //待写入的标签数据区(0，保留区；1，EPC 区；2，TID 区；3，用户数据区)
    unsigned short Start; //待写入标签数据区的字起始地址
    //char* StrHexWriteData; //待写入的数据内容(16进制字符串)
    char StrHexWriteData[1024];
    ParamFilter Filter; //选择读取参数（详见参数说明）
    char StrHexPassword[9]; //访问密码, 16进制字符串
    int Block; //块写参数，0表示不块写
    int StayCarrierWave; //操作完成保持载波发射状态
} MsgBaseWriteEpc;
#endif

#if REGION("锁EPC")
typedef struct
{
    Result rst;
    int AntennaEnable; //天线端口（使用天线枚举，详见AntennaNo,多个天线使用或）
    char Area; //待锁定的标签数据区(0，灭活密码区；1，访问密码区；2，EPC 区；3，TID 区；4，用户数据区)
    char Mode; //锁操作类型(0，解锁；1，锁定；2，永久解锁；3，永久锁定)
    ParamFilter Filter; //选择读取参数（详见参数说明）
    char StrHexPassword[9]; //访问密码,, 16进制字符串
} MsgBaseLockEpc;
#endif

#if REGION("灭活标签")
typedef struct
{
    Result rst;
    int AntennaEnable; //天线端口（使用天线枚举，详见AntennaNo,多个天线使用或）
    char StrHexPassword[9]; //密码,16进制字符串
    ParamFilter Filter; //选择读取参数（详见参数说明）
} MsgBaseDestroyEpc;
#endif

#if REGION("MONZA QT标签操作")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char QtOperate;

    ParamFilter Filter;

    char StrHexPassword[9];

    bool setQtParamFlag;
    unsigned char QtParam[3];

    bool setQtResultFlag;
    unsigned char QtResult[3];
} MsgBaseMonzaQt;
#endif

#if REGION("EPC标签超级读写指令")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Operate;
    char Area[3];
    char SpecialCode[3];

    bool setWriteContentFlag;
    unsigned char WriteContent[3];

    bool setReadContentFlag;
    unsigned char ReadContent[3];
} MsgBaseSuperRW;
#endif

#if REGION("读6B标签")
typedef struct
{
    char Start;
    char Len;
} Param6bReadUserdata;

typedef struct
{
    Result rst;
    int AntennaEnable;
    char InventoryMode;
    char Area;

    bool setReadUserdataFlag;
    Param6bReadUserdata ReadUserdata;

    bool setMatchTidFlag;
    char TidLenth;
    char StrHexMatchTid[1024];
} MsgBaseInventory6B;
#endif

#if REGION("写6B标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char TidLength;
    char StrHexMatchTid[1024];
    char Start;
    char DataLength;
    char StrHexWriteData[1024];

    bool setErrorIndexFlag;
    char ErrorIndex;
} MsgBaseWrite6B;
#endif

#if REGION("锁6B标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char TidLength;
    char StrHexMatchTid[1024];
    unsigned char Address;

    bool setErrorAddressFlag;
    char ErrorAddress;
} MsgBaseLock6B;
#endif

#if REGION("锁定查询6B标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char TidLength;
    char StrHexMatchTid[1024];
    unsigned char Address;

    bool setLockStateFlag;
    char LockState;
} MsgBaseLockGet6B;
#endif

#if REGION("读GB标签")
typedef struct
{
    Result rst;
    unsigned char ChildArea;
    unsigned short Start;
    unsigned char Len;
} ParamGbReadUserdata;

typedef struct
{
    Result rst;
    int AntennaEnable;
    char InventoryMode;
    ParamFilter Filter;
    ParamReadTid ReadTid;
    ParamGbReadUserdata ReadUserdata;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseInventoryGB;
#endif

#if REGION("写GB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Area;
    unsigned short Start;
    char DataLength;
    //char* DataContent;
    char DataContent[1024];
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
    bool setFDMicroInitFlag;
    char FDMicroInit;
    bool setStayCarrierWaveFlag;
    char StayCarrierWave;
} MsgBaseWriteGB;
#endif

#if REGION("锁GB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Area;
    char LockParam;
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseLockGB;
#endif

#if REGION("灭活GB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseDestroyGB;
#endif

#if REGION("读GJB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char InventoryMode;
    ParamFilter Filter;
    ParamReadTid ReadTid;
    ParamReadUserdata ReadUserdata;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseInventoryGJB;
#endif

#if REGION("写GJB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Area;
    unsigned short Start;
    char DataLength;
    //char* DataContent;
    char DataContent[1024];
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
    bool setStayCarrierWaveFlag;
    char StayCarrierWave;
} MsgBaseWriteGJB;
#endif

#if REGION("锁GJB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Area;
    char LockParam;
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseLockGJB;
#endif

#if REGION("灭活GJB标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    ParamFilter Filter;
    char StrHexPassword[9];
    bool setSafeCertificationFlag;
    char SafeCertificationFlag;
} MsgBaseDestroyGJB;
#endif

#if REGION("读GB/T25340标签")
typedef struct
{
    Result rst;
    int tagType;
    int AntennaEnable;
    char InventoryMode;
    int Timeout;
} MsgBaseInventoryTL;
#endif

#if REGION("读6D标签")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char InventoryMode;
    int Timeout;
} MsgBaseInventory6D;
#endif

#if REGION("标签安全认证数据交互指令")
typedef struct
{
    short bitLen;
    char Content[256];
} EncryptedData;

typedef struct
{
    Result rst;
    bool setToken1Flag;
    unsigned char Token1[9];
    bool setToken2Flag;
    char Token2;
    bool setEncryptedDataFlag;
    EncryptedData Data;
    bool setKeyFlag;
    char Key[512];
} MsgBaseSafeCertification;
#endif

#if REGION("多协议组合读")
typedef struct
{
    Result rst;
    int AntennaEnable;
    char Read6B;
    char ReadGB;
} MsgBaseInventoryHybrid;
#endif

#if REGION("基带软件透传数据消息")
typedef struct
{
    Result rst;
    char Mode;
    char DataLength;
    //char* DataContent;
    char DataContent[1024];
} MsgBasePassthrough;

#endif

#if REGION("停止")
typedef struct
{
    Result rst;
} MsgBaseStop;
#endif

#if REGION("应用软件升级")
typedef struct
{
    Result rst;
    unsigned long packetNumber;
    char packetContent[1024];
} MsgUpgradeApp;
#endif

#if REGION("基带软件升级")
typedef struct
{
    Result rst;
    unsigned long packetNumber;
    char packetContent[1024];
} MsgUpgradeBaseband;
#endif

#if REGION("模块工作模式初始化")
typedef struct
{
    Result rst;
    int len;
    char license[1024];
    char mode;
} MsgTestWorkingMode;
#endif

#ifdef __cplusplus
}
#endif

#endif //GREADER_MESSAGE_H
