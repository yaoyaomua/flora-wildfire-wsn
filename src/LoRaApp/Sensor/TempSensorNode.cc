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

#include "TempSensorNode.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

// namespace std {


TempSensorNode::TempSensorNode() {
    // TODO Auto-generated constructor stub
    this->sensorType = 1;
    this->lastData = ((rand() % 5000 / 1000.0) + 15.001);
}

TempSensorNode::TempSensorNode(double inidata) {
    // TODO Auto-generated constructor stub
    this->sensorType = 1;
    this->lastData = inidata;
}

TempSensorNode::~TempSensorNode() {
    // TODO Auto-generated destructor stub
}

double TempSensorNode :: getData(){
    if (lastData < -20){
        lastData += ((rand() % 5000 / 1000.0 ) + 1.001);
    }else if (lastData < 60){
        if(rand() % 1000 / 1000.0 > 0.005 ){
            lastData += ((rand() % 2000 / 1000.0 ) - 1.001);
        }else{
            lastData += ((rand() % 50000 / 1000.0) + 70.001);
        }
    }else{
        if ( rand() % 1000 / 1000.0 > 0.3 && lastData < 800.0){
            lastData += ((rand() % 10000 / 1000.0) + 10.001);
        }else{
            lastData += ((rand() % 10000 / 1000.0) - 5.001);
        }
    }

    return lastData;
}

double TempSensorNode :: forceFire(){
    if(lastData < 100){
        lastData = ((rand() % 20000 / 1000.0) + 100.001);
    }
    return lastData;
}

int TempSensorNode :: getType(){
    return sensorType;
}

void TempSensorNode :: setLastData(double data){
    this->lastData = data;
}

double TempSensorNode :: getLastData(){
    return lastData;
}
