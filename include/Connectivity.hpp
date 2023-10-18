#include <WiFiUdp.h>

extern WiFiUDP udp;
extern bool connected;
char* udpReceive();
void udpSend(char bufferSend[]);
void udpPrepareMessage(int subject);
void maintainWifiConnection();