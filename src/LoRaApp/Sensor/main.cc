#include "TempSensorNode.h"
#include "HumiditySensorNode.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

int main(){

    // HumiditySensor *hs = new HumiditySensor();
    double iniTemp = 20.0;
    TempSensorNode *ts = new TempSensorNode(iniTemp);
    HumiditySensorNode *hs = new HumiditySensorNode();
    // double iniHum = 60;
    srand((unsigned)time(NULL));
    for (int i = 0; i < 100; i++)
    {
        std::cout<< i <<std::endl;
        iniTemp = ts->getData();
        std::cout<< iniTemp <<std::endl;

        iniTemp = hs->getData();
        std::cout<< iniTemp <<std::endl;
    }

    iniTemp = ts->forceFire();
    std::cout<< iniTemp <<std::endl;

    delete ts;
    return 0;
}