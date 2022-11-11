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

#ifndef LORAAPP_SENSOR_SENSORNODE_H_
#define LORAAPP_SENSOR_SENSORNODE_H_
// #include <omnetpp.h>
// #include <iostream>
// #include <stdlib.h>
// #include <time.h>
// #include<iostream>
// #include<stdlib.h>

// using namespace omnetpp;
using namespace std;
// namespace std {

class SensorNode  {
public:
    virtual void setLastData(double data) = 0;
    virtual int  getType() = 0;
    virtual double getLastData() = 0;
    virtual double getData() = 0;
    virtual double forceFire() = 0;
    // enum sensorType{
    //     TEMP = 1,
    //     HUMI
    // };

//    functions dont need to override
    // void setLastData(double data);
    // int  getType();
    // double getLastData();
    // double getRam(int a, int b);


    // virtual double getRam(int a, int b) = 0;


    // SensorNode();
    // virtual ~SensorNode();

//virtual functions



};

// } /* namespace flora */

#endif /* LORAAPP_SENSOR_SENSORNODE_H_ */
