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

package flora.LoRaEnergyModules;

import inet.power.contract.IEpEnergyConsumer;

simple LoRaAppEnergyConsumer like IEpEnergyConsumer
{
    parameters:
        string energySourceModule;
        double sleepAppPowerConsumption @unit(W) = default(1mW);
        double runAppPowerConsumption @unit(W) = default(100mW);
        @class(LoRaAppEnergyConsumer);
        @signal[powerConsumptionChanged](type=double);
        @statistic[powerConsumption](title="Power consumption"; source=powerConsumptionChanged; record=vector; interpolationmode=sample-hold);
}
