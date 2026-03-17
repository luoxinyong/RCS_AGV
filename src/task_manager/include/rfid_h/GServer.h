#ifndef GREADER_GSERVER_H
#define GREADER_GSERVER_H

#include "GClient.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*delegateGClientConnected)(char *readerName, GClient *client);

typedef struct {
    int handle;
    delegateGClientConnected call_GClientConnected;
} GServer;

void RegGServerCallBack(GServer *s, void *call);

bool OpenTcpServer(short port, GServer *server);
bool CloseTcpServer(GServer *server);

#ifdef __cplusplus
}
#endif

#endif //GREADER_GSERVER_H
