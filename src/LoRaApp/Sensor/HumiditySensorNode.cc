//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "HumiditySensorNode.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

// namespace flora {

//Define_Module(HumiditySensorNode);

HumiditySensorNode::HumiditySensorNode() {
    // TODO Auto-generated constructor stub
    this->sensorType = 2;
    this->lastData = ((rand() % 5000 / 1000.0) + 15.001);
}

HumiditySensorNode::HumiditySensorNode(double inidata) {
    // TODO Auto-generated constructor stub
    this->sensorType = 2;
    this->lastData = inidata;
}


HumiditySensorNode::~HumiditySensorNode() {
    // TODO Auto-generated destructor stub
}

double HumiditySensorNode :: getData(){
    if (lastData < 10)
    {
        lastData += (rand() % 2000 / 1000.0 );
    }else if (lastData > 90)
    {
        lastData -= (rand() % 2000 / 1000.0 );
    }else{
        if (lastData > 30)
        {
            if (rand() % 1000 / 1000.0 > 0.005)
            {
                lastData +=((rand() % 5000 / 1000.0) + 2.5);
            }else{
                lastData +=((rand() % 5000 / 1000.0) - 10.001);
            }
        }else{
            lastData +=((rand() % 5000 / 1000.0) - 2.5);
        }


    }

    return lastData;

}

double HumiditySensorNode :: forceFire(){
    if(lastData > 15){
        lastData = ((rand() % 20000 / 1000.0) + 100.001)                         ;
    }
    return lastData;
}


int HumiditySensorNode :: getType(){
    return sensorType;
}

void HumiditySensorNode :: setLastData(double data){
    this->lastData = data;
}

double HumiditySensorNode :: getLastData(){
    return lastData;
}


 /* namespace flora */



//}
