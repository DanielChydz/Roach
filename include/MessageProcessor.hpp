#ifndef ROACH_MSGPROC_H
#define ROACH_MSGPROC_H
#include "Config.hpp"
#include <ctype.h> // isdigit
#include <string.h> // strcmp
#include <esp_log.h>

void processMessage(char* msg);

#endif