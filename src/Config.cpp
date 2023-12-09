#include "Config.hpp"

const char keyWord[18] = "DC_Remote_Car_Key";

connectivityData connData = {
    .udpSendAddress = "192.168.220.201",
    .udpSendPort = 4322,
    .udpReceivePort = 4321,
};

taskConfig printDotsWhileConnectingToWifiConfig = {
    .taskLoopDelay = 1000,
    .taskPriority = 5,
    .taskCore = 1,
    .taskHandle = NULL
};

taskConfig udpClientConfig = {
    .taskLoopDelay = 2000,
    .taskPriority = 2,
    .taskCore = 1,
};

taskConfig udpServerConfig = {
    .taskLoopDelay = 200,
    .taskPriority = 2,
    .taskCore = 1,
};

taskConfig wifiServiceCheckConnectionConfig = {
    .taskLoopDelay = 100,
    .taskSecondLoopDelay = 250,
    .taskPriority = 3,
    .taskCore = 1,
};

taskConfig motorServiceConfig = {
    .taskLoopDelay = 200,
    .taskPriority = 1,
    .taskCore = 0,
};