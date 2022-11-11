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

package flora.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import inet.applications.contract.IApp;
import flora.LoRa.LoRaNic;
import inet.power.contract.IEnergyConsumer;
import inet.power.contract.IEnergyStorage;
import inet.common.lifecycle.NodeStatus;


module LoRaNode
{
    parameters:
        int numApps = default(0);
        string deploymentType = default("");
        double minX @unit(m) = default(0m);
        double maxX @unit(m) = default(1000m);
        double sepX @unit(m) = default(1000m);
        double minY @unit(m) = default(0m);
        double maxY @unit(m) = default(1000m);
        double sepY @unit(m) = default(1000m);
        int cols = default(10);
        double maxGatewayDistance = default(320.0);
        double gatewayX = default(320.0);
        double gatewayY = default(320.0);
        bool hasStatus = default(false);

        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=496,313;i=device/lora_node;is=vs");
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=59,37");
        }
        mobility: StationaryMobilityMut {
            @display("p=59,128");
        }
        LoRaNic: LoRaNic {
            @display("p=137,239");
        }
        app[numApps]: <> like IApp {
            parameters:
                @display("p=375,76,row,150");
        }
        appEnergyConsumer: <default("")> like IEnergyConsumer if typename != "" {
            @display("p=393,219");
        }
        energyStorage: <default("")> like IEnergyStorage if typename != "" {
            @display("p=125,400;is=s");
        }
        status: NodeStatus if hasStatus {
            @display("p=100,100;is=s");
        }
    connections allowunconnected:
        for i=0..numApps-1 {
            app[i].socketOut --> LoRaNic.upperLayerIn;
            LoRaNic.upperLayerOut --> app[i].socketIn;
        }
}
