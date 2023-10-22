extern bool connected;
void udpSend(char bufferSend[]);
void udpPrepareMessage(int subject);
void connectWifi();
void receiveUDPPacket(void *params);