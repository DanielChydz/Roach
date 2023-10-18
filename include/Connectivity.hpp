#include <WiFiUdp.h>

extern WiFiUDP udp;
extern bool connected;
void udpSend(char bufferSend[]);
void udpPrepareMessage(int subject);
void maintainWifiConnection(void *param);
void receiveUDPPacket(void *params);