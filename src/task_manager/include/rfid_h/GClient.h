#ifndef GREADER_GCLIENT_H
#define GREADER_GCLIENT_H


#include "delegate.h"
#include "message.h"
#include <semaphore.h>
#include <string>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    EMESS_AppGetReaderInfo = 0,
    EMESS_AppGetBaseVersion = 1,
    EMESS_AppSetSerialParam = 2,
    EMESS_AppGetSerialParam = 3,
    EMESS_AppSetEthernetIp = 4,
    EMESS_AppGetEthernetIp = 5,
    EMESS_AppGetEthernetMac = 6,
    EMESS_AppSetTcpMode = 7,
    EMESS_AppGetTcpMode = 8,
    EMESS_AppSetGpo = 9,
    EMESS_AppGetGpiState = 10,
    EMESS_AppSetGpiTrigger = 11,
    EMESS_AppGetGpiTrigger = 12,
    EMESS_AppSetWiegand = 13,
    EMESS_AppGetWiegand = 14,
    EMESS_AppReboot = 15,
    EMESS_AppSetReaderTime = 16,
    EMESS_AppGetReaderTime = 17,
    EMESS_AppHeartbeat = 18,
    EMESS_AppSetReaderMac = 19,
    EMESS_AppRestoreDefault = 20,
    EMESS_AppSetRs485 = 21,
    EMESS_AppGetRs485 = 22,
    EMESS_AppSetBreakpointResume = 23,
    EMESS_AppGetBreakpointResume = 24,
    EMESS_AppGetCacheTagData = 25,
    EMESS_AppClearCacheTagData = 26,
    EMESS_AppTagDataReply = 27,
    EMESS_AppSetBuzzerSwitch = 28,
    EMESS_AppSetBuzzerCtrl = 29,
    EMESS_AppGetWhiteList = 30,
    EMESS_AppImportWhiteList = 31,
    EMESS_AppDeleteWhiteList = 32,
    EMESS_AppSetWhiteListAction = 33,
    EMESS_AppGetWhiteListAction = 34,
    EMESS_AppSetWhiteListSwitch = 35,
    EMESS_AppGetWhiteListSwitch = 36,
    EMESS_AppSetUdpParam = 37,
    EMESS_AppGetUdpParam = 38,
    EMESS_AppSetHttpParam = 39,
    EMESS_AppGetHttpParam = 40,
    EMESS_AppUsbKbdSwitch = 41,
    EMESS_AppSetWifiSearch = 42,
    EMESS_AppGetWifiSearch = 43,
    EMESS_AppSetWifiParam = 44,
    EMESS_AppGetWifiStatus = 45,
    EMESS_AppSetWifiIp = 46,
    EMESS_AppGetWifiIp = 47,
    EMESS_AppSetWifiOnOff = 48,
    EMESS_AppGetWifiOnOff = 49,
    EMESS_AppSetEasAlarm = 50,
    EMESS_AppGetEasAlarm = 51,
    EMESS_AppWifiRoaming = 52,
    EMESS_AppWatchDog = 53,

    EMESS_BaseGetCapabilities = 54,
    EMESS_BaseSetPower = 55,
    EMESS_BaseGetPower = 56,
    EMESS_BaseSetFreqRange = 57,
    EMESS_BaseGetFreqRange = 58,
    EMESS_BaseSetFrequency = 59,
    EMESS_BaseGetFrequency = 60,
    EMESS_BaseSetAntennaHub = 61,
    EMESS_BaseGetAntennaHub = 62,
    EMESS_BaseSetTagLog = 63,
    EMESS_BaseGetTagLog = 64,
    EMESS_BaseSetBaseband = 65,
    EMESS_BaseGetBaseband = 66,
    EMESS_BaseSetAutoDormancy = 67,
    EMESS_BaseGetAutoDormancy = 68,
    EMESS_BaseSetResidenceTime = 69,
    EMESS_BaseGetResidenceTime = 70,
    EMESS_BaseSetGbBaseband = 71,
    EMESS_BaseGetGbBaseband = 72,
    EMESS_BaseSetGjbBaseband = 73,
    EMESS_BaseGetGjbBaseband = 74,
    EMESS_BaseInventoryEpc = 75,
    EMESS_BaseWriteEpc = 76,
    EMESS_BaseLockEpc = 77,
    EMESS_BaseDestroyEpc = 78,
    EMESS_BaseMonzaQt = 79,
    EMESS_BaseSetMultiFilter = 80,
    EMESS_BaseGetMultiFilter = 81,
    EMESS_BaseSuperRW = 82,
    EMESS_BaseInventory6b = 83,
    EMESS_BaseWrite6b = 84,
    EMESS_BaseLock6b = 85,
    EMESS_BaseLockGet6b = 86,
    EMESS_BaseInventoryGb = 87,
    EMESS_BaseWriteGb = 88,
    EMESS_BaseLockGb = 89,
    EMESS_BaseDestroyGb = 90,
    EMESS_BaseInventoryGjb = 91,
    EMESS_BaseWriteGjb = 92,
    EMESS_BaseLockGjb = 93,
    EMESS_BaseDestroyGjb = 94,
    EMESS_BaseInventoryTL = 95,
    EMESS_BaseInventory6D = 96,
    EMESS_BaseSafeCertification = 97,
    EMESS_BaseInventoryHybrid = 98,
    EMESS_BasePassthrough = 99,
    EMESS_BaseStop = 100,

    EMESS_UpgradeApp = 101,
    EMESS_UpgradeBaseband = 102,

    EMESS_TestInitWorkingMode = 103,

    EMESS_Count
} MESSAGE;

typedef enum
{
    ETagEpcLog = 0,
    ETagEpcOver = 1,
    ETag6bLog = 2,
    ETag6bOver = 3,
    ETagGbLog = 4,
    ETagGbOver = 5,
    ETagGjbLog = 6,
    ETagGjbOver = 7,
    EGpiStart = 8,
    EGpiOver = 9,
    ETcpDisconnected = 10,
    EGClientConnected = 11,
    EUsbHidRemoved = 12,
    ETagTLLog = 13,
    ETagTLOver = 14,
    ETag6DLog = 15,
    ETag6DOver = 16,
} Callback_Type;

typedef enum
{
    RS232 = 1,
    RS485 = 2,
    ETH = 3,
    SERVER = 4,
    ACCEPT = 5,
    USB = 6,
    OTHER = 1000
} ConnType;

typedef struct
{
    void* rst;
} MessageResult;

typedef void(*delegateOnRemoteConnected)(char *readerName, int handleSocket);

typedef struct {
    long handleSocket;
    delegateOnRemoteConnected onRemoteConnected;
}TcpServer;

typedef struct
{
    char name[128];
    char serialNumber[21];
    long handle;
    delegateTagEpcLog call_TagEpcLog;
    delegateTagEpcOver call_TagEpcOver;
    delegateTag6bLog call_Tag6bLog;
    delegateTag6bOver call_Tag6bOver;
    delegateTagGbLog call_TagGbLog;
    delegateTagGbOver call_TagGbOver;
    delegateTagGjbLog call_TagGjbLog;
    delegateTagGjbOver call_TagGjbOver;
    delegateTagTLLog call_TagTLLog;
    delegateTagTLOver call_TagTLOver;
    delegateTag6DLog call_Tag6DLog;
    delegateTag6DOver call_Tag6DOver;
    delegateGpiStart call_GpiStart;
    delegateGpiOver call_GpiOver;
    delegateWorkingModeInit call_WorkingModeInit;
    delegateTcpDisconnected call_TcpDisconnected;
    delegateUsbHidRemoved call_UsbHidRemoved;
    sem_t* sem;
    MessageResult* result;
    pthread_mutex_t mutex;
    TcpServer* tcpServer;
    int initParamLen;
    char initParam[1024];
    unsigned char* data;
    int index;
    ConnType type;
    int rs485Address;
    int heartbeatTimeout;
    bool isPrint;
    bool isOpened;
    bool threadIsStop;
    bool timerThreadIsStop;
    unsigned long tick;
    pthread_t tid;
} GClient;

GClient* OpenRS232(const char* readerName, int timeout);
GClient* OpenRS485(const char* readerName, int timeout);
GClient* OpenTcpClient(const char* readerName, int timeout);
int GetAttachedUsbHid(std::vector<std::string>* devs_path);
GClient* OpenUSBHID(const char* readerName, int timeout);
void RegCallBack(GClient* client, Callback_Type type, void* call);
void CancelCallBack(GClient* client);
void SendSynMsgTimeout(GClient* client, MESSAGE type, void* msg, int timeout);
void SendSynMsg(GClient* client, MESSAGE type, void* msg);
void SendSynMsgTimeoutRetry(GClient* client, MESSAGE type, void* msg, int timeout, int retry);
void SendUnSynMsg(GClient* client, MESSAGE type, void* msg);
void SendUnSynMsgTimeoutRetry(GClient* client, MESSAGE type, void* msg, int retry);
void Close(GClient* client);

#ifdef __cplusplus
}
#endif

#endif //GREADER_GCLIENT_H
