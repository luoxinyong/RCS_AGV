#ifndef GREADER_DELEGATE_H
#define GREADER_DELEGATE_H

//6C
typedef struct
{
    unsigned char Epc[1024]; //16进制EPC 字符串
    unsigned char Pc[5]; //PC 值, 16进制字符串
    unsigned char AntId; //天线编号
    unsigned char Rssi; //信号强度
    unsigned char Result; //标签读取结果，0 为读取成功，非0 为失败	1，标签无响应	2，CRC 错误	3，数据区被锁定	4，数据区溢出	5，访问密码错误	6，其他标签错误	7，其他读写器错误
    unsigned char Tid[512]; //16 进制TID 字符串
    unsigned char Userdata[1024]; //16进制Userdata 字符串
    unsigned char Reserved[512]; //16进制保留区字符串
    unsigned char SubAntId; //子天线号
    int ReadUtcSecTime; //UTC时间秒
    int ReadUtcMicroSecTime; //UTC时间微秒
    int Point; //当前频点
    unsigned char Phase; //当前标签相位
    unsigned char EpcBank[1024]; //EPC区数据
    int CtesiusLtu27; //CTESIUS LTU27温度传感数据
    int CtesiusLtu31; //CTESIUS LTU31温度传感数据
    int Quanray; //坤锐温度传感数据
    short RSSI_dBm;
    unsigned char CRC[3];
    int ReplySerialNumber; //标签应答包序号
    char readerSerialNumber[21]; //设备序列号
} LogBaseEpcInfo;

typedef void (*delegateTagEpcLog)(char* readerName, LogBaseEpcInfo msg);

typedef struct
{
} LogBaseEpcOver;

typedef void (*delegateTagEpcOver)(char* readerName, LogBaseEpcOver msg);

//6B
typedef struct
{
    unsigned char Tid[18];
    unsigned char AntId;
    unsigned char Rssi;
    unsigned char Result;
    unsigned char Userdata[512];
    unsigned char DeviceSerial[64];
} LogBase6bInfo;

typedef void (*delegateTag6bLog)(char* readerName, LogBase6bInfo msg);

typedef struct
{
} LogBase6bOver;

typedef void (*delegateTag6bOver)(char* readerName, LogBase6bOver msg);

//GB
typedef struct
{
    unsigned char Epc[1024];
    unsigned char Pc[3];
    unsigned char AntId;
    unsigned char Rssi;
    unsigned char Result;
    unsigned char Tid[512];
    unsigned char Userdata[1024];
    unsigned char DeviceSerial[64];
} LogBaseGbInfo;

typedef void (*delegateTagGbLog)(char* readerName, LogBaseGbInfo msg);

typedef struct
{
} LogBaseGbOver;

typedef void (*delegateTagGbOver)(char* readerName, LogBaseGbOver msg);

//GJB
typedef struct
{
    unsigned char Epc[1024];
    unsigned char Pc[3];
    unsigned char AntId;
    unsigned char Rssi;
    unsigned char Result;
    unsigned char Tid[512];
    unsigned char Userdata[1024];
    unsigned char DeviceSerial[64];
} LogBaseGjbInfo;

typedef void (*delegateTagGjbLog)(char* readerName, LogBaseGjbInfo msg);

typedef struct
{
} LogBaseGjbOver;

typedef void (*delegateTagGjbOver)(char* readerName, LogBaseGjbOver msg);

//GB/T25340
typedef struct
{
    int tagType;
    unsigned char Epc[1024];
    unsigned char Tid[1024];
    int AntId;
    int Rssi;
    unsigned char SubAntId; //子天线号
    int ReadUtcSecTime; //UTC时间秒
    int ReadUtcMicroSecTime; //UTC时间微秒
    int Point; //当前频点
    unsigned char Phase; //当前标签相位
    int RSSI_dBm;
    int ReplySerialNumber; //标签应答包序号
    char readerSerialNumber[21]; //设备序列号
} LogBaseTLInfo;

typedef void (*delegateTagTLLog)(char* readerName, LogBaseTLInfo msg);

typedef struct
{
    char readerSerialNumber[21]; //设备序列号
} LogBaseTLOver;

typedef void (*delegateTagTLOver)(char* readerName, LogBaseTLOver msg);

//6D
typedef struct
{
    int tagType;
    unsigned char Epc[1024];
    int AntId;
    int Rssi;
    unsigned char SubAntId; //子天线号
    int ReadUtcSecTime; //UTC时间秒
    int ReadUtcMicroSecTime; //UTC时间微秒
    int Point; //当前频点
    unsigned char Phase; //当前标签相位
    int RSSI_dBm;
    int ReplySerialNumber; //标签应答包序号
    char readerSerialNumber[21]; //设备序列号
} LogBase6DInfo;

typedef void (*delegateTag6DLog)(char* readerName, LogBase6DInfo msg);

typedef struct
{
    char readerSerialNumber[21]; //设备序列号
} LogBase6DOver;

typedef void (*delegateTag6DOver)(char* readerName, LogBase6DOver msg);

//GPI
typedef struct
{
    unsigned char GpiPort;
    unsigned char Level;
    char TriggerTime[32];
} LogAppGpiStart;

typedef void (*delegateGpiStart)(char* readerName, LogAppGpiStart msg);

typedef struct
{
    unsigned char GpiPort;
    unsigned char Level;
    char TriggerTime[32];
} LogAppGpiOver;

typedef void (*delegateGpiOver)(char* readerName, LogAppGpiOver msg);

typedef struct
{
    unsigned char reason;
} LogTestWorkingModeInit;

typedef void (*delegateWorkingModeInit)(char* readerName, LogTestWorkingModeInit msg);

//TCP
typedef void (*delegateTcpDisconnected)(char* readerName);

typedef void (*delegateUsbHidRemoved)(char* readerName);

#endif //GREADER_DELEGATE_H
