#include <WiFiUdp.h>

extern WiFiUDP udp;
extern bool connected;
char* udpReceive();
void udpSend(char bufferSend[]);
void runMaintainWifiConnectionRoutine();
void runReceiveUDPRoutine();
void udpPrepareMessage(int subject);