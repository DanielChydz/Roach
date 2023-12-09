#ifndef ROACH_CONN_H
#define ROACH_CONN_H
#include "Config.hpp"

extern bool connected;

void udpPreparePayload(int subject);
void startWifiService();

#endif