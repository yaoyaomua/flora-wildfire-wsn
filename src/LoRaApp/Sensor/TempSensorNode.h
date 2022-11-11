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

#ifndef LORAAPP_SENSOR_TEMPSENSORNODE_H_
#define LORAAPP_SENSOR_TEMPSENSORNODE_H_
#include "SensorNode.h"

// namespace std {

class TempSensorNode : public SensorNode  {
private:
    double lastData;
    int sensorType;
public:
    // virtual void initialize(int stage) override;
    TempSensorNode();
    TempSensorNode(double inidata);
    ~TempSensorNode();
    double getData() override;
    double forceFire() override;
    void setLastData(double data) override;
    int  getType() override;
    double getLastData() override;
};

// } /* namespace flora */

#endif /* LORAAPP_SENSOR_TEMPSENSORNODE_H_ */
