#ifndef ROACH_CONN_H
#define ROACH_CONN_H
#include "Config.hpp"
#include <esp_timer.h>

extern bool connected, lastMeasure;

void startWifiService();

#endif