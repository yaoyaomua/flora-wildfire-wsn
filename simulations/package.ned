package flora.simulations;

import inet.applications.udpapp.UdpBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import flora.LoRaPhy.LoRaMedium;
import flora.LoraNode.LoRaNode;
import flora.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.visualizer.contract.IIntegratedVisualizer;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=355.824,356.832");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=167.328,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=208.24002,27.216002");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.64801,88.704");
        }
        internetCloud: InternetCloud {
            @display("p=83.664,27.216002");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=24.192001,27.216002");
        }
        nsRouter: Router {
            @display("p=138.09601,27.216002");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

network LoRaMotoNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        //@display("bgb=1400,2500;bgi=background/coquimbo-02;bgl=2");
        @display("bgb=2500,2500;bgl=2");
    submodules:
        N[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856;is=n");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=100,100;is=n");
        }
        LoRaMedium: LoRaMedium {
            @display("p=20,35;is=l;i=misc/sun");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.64801,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=208.24002,27.216002");
        }
    connections:
}


network LoRaMesh
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        //@display("bgb=1400,2500;bgi=background/coquimbo-02;bgl=2");
        @display("bgb=2500,2500;bgl=2");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856;is=n");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=200,200");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=850,-100");
        }
        visualizer: <default(firstAvailableOrEmpty("IntegratedCanvasVisualizer"))> like IIntegratedVisualizer if typename != "" {
            @display("p=580,125");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=1200,-100");
        }
        internetCloud: InternetCloud {
            @display("p=300,-100");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=50,-100");
        }
        nsRouter: Router {
            @display("p=550,-100");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}
