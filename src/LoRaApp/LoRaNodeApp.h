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

#ifndef __LORA_OMNET_LORANODEAPP_H_
#define __LORA_OMNET_LORANODEAPP_H_

#include <omnetpp.h>
#include <string>

#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "inet/common/FSMA.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "LoRa/LoRaRadio.h"

using namespace omnetpp;
using namespace inet;

namespace flora {

/**
 * TODO - Generated class
 */
class LoRaNodeApp : public cSimpleModule, public ILifecycle
{
    public:
        LoRaNodeApp() {}
        simsignal_t LoRa_AppPacketSent;

        static simsignal_t appModeChangedSignal;

        enum AppMode {
            APP_MODE_SLEEP,
            APP_MODE_RUN,
            APP_MODE_SWITCHING // this mode must be the very last
        };

        virtual AppMode getAppMode() const  { return appMode; }
        virtual void setAppMode(AppMode newAppMode);
        static const char *getAppModeName(AppMode appMode);


    protected:
        virtual void initialize(int stage) override;
        virtual void initializeAppMode();

        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

        virtual bool isNeighbour(int neighbourId);
        virtual bool isRouteInSingleMetricRoutingTable(int id, int via);
        virtual int  getRouteIndexInSingleMetricRoutingTable(int id, int via);
        virtual bool isRouteInDualMetricRoutingTable(int id, int via, int sf);
        virtual int  getRouteIndexInDualMetricRoutingTable(int id, int via, int sf);
        virtual bool isKnownNode(int knownNodeId);
        virtual bool isACKed(int nodeId);
        virtual bool isPacketForwarded(const LoRaAppPacket & packet);
        virtual bool isPacketToBeForwarded(const LoRaAppPacket & packet);
        virtual bool isDataPacketForMeUnique(const LoRaAppPacket & packet);

        void handleMessageFromLowerLayer(cMessage *msg);
        void handleSelfMessage(cMessage *msg);

        simtime_t getTimeToNextRoutingPacket();

        simtime_t sendRoutingPacket();
        simtime_t sendDataPacket();
        void manageReceivedPacketForMe(cMessage *msg);
        void manageReceivedAckPacketForMe(cMessage *msg);
        void manageReceivedDataPacketForMe(cMessage *msg);
        void manageReceivedPacketToForward(cMessage *msg);
        void manageReceivedAckPacketToForward(cMessage *msg);
        void manageReceivedDataPacketToForward(cMessage *msg);
        void manageReceivedRoutingPacket(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendDownMgmtPacket();
        void sanitizeRoutingTable();
        int pickCADSF(int lowestSF);
        int getBestRouteIndexTo(int destination);
        int getSFTo(int destination);

        simtime_t calculateTransmissionDuration(cMessage *msg);

        bool sendPacketsContinuously;
        bool onlyNode0SendsPackets;
        bool enforceDutyCycle;
        double dutyCycle;
        int numberOfDestinationsPerNode;
        int numberOfPacketsPerDestination;

        int numberOfPacketsToForward;

        int sentPackets;
        int sentDataPackets;
        int sentRoutingPackets;
        int sentAckPackets;
        int receivedPackets;
        int receivedPacketsForMe;
        int receivedPacketsFromMe;
        int receivedPacketsToForward;
        int receivedDataPackets;
        int receivedDataPacketsForMe;
        int receivedDataPacketsForMeUnique;
        int receivedDataPacketsFromMe;
        int receivedDataPacketsToForward;
        int receivedDataPacketsToForwardCorrect;
        int receivedDataPacketsToForwardExpired;
        int receivedDataPacketsToForwardUnique;
        int receivedAckPackets;
        int receivedAckPacketsForMe;
        int receivedAckPacketsFromMe;
        int receivedAckPacketsToForward;
        int receivedAckPacketsToForwardCorrect;
        int receivedAckPacketsToForwardExpired;
        int receivedAckPacketsToForwardUnique;
        int receivedRoutingPackets;
        int receivedADRCommands;
        int forwardedPackets;
        int forwardedDataPackets;
        int forwardedAckPackets;
        int forwardPacketsDuplicateAvoid;
        int broadcastDataPackets;
        int broadcastForwardedPackets;
        int lastSentMeasurement;
        int deletedRoutes;
        int forwardBufferFull;

        simtime_t timeToFirstRoutingPacket;
        std::string timeToNextRoutingPacketDist;
        simtime_t timeToNextRoutingPacketMin;
        simtime_t timeToNextRoutingPacketMax;
        simtime_t timeToNextRoutingPacketAvg;

        simtime_t timeToFirstTaskTimerTick;
        simtime_t timeToNextTaskTimerTick;

        simtime_t nextRoutingPacketTransmissionTime;
        simtime_t nextDataPacketTransmissionTime;
        simtime_t nextForwardPacketTransmissionTime;
        simtime_t minNextPacketTransmissionTime;

        bool dataPacketsDue;
        bool routingPacketsDue;

        cHistogram allTxPacketsSFStats;
        cHistogram routingTxPacketsSFStats;
        cHistogram owndataTxPacketsSFStats;
        cHistogram fwdTxPacketsSFStats;

        cHistogram dataPacketsForMeLatency;
        cHistogram dataPacketsForMeUniqueLatency;

        cHistogram routingTableSize;

        simtime_t firstDataPacketTransmissionTime;
        simtime_t lastDataPacketTransmissionTime;
        simtime_t firstDataPacketReceptionTime;
        simtime_t lastDataPacketReceptionTime;

        simtime_t simTimeResolution;

        cMessage *selfPacketTxMsg;
        cMessage *selfTaskTimerMsg;

        //history of sent packets;
        cOutVector txSfVector;
        cOutVector txTpVector;

        // History of received packets
        cOutVector rxRssiVector;
        cOutVector rxSfVector;

        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

        int currDataInt;

        //General network variables
        int numberOfNodes;

        //Packet sizes
        int dataPacketSize;
        int routingPacketMaxSize;

        //Routing variables
        int routingMetric;
        bool routeDiscovery;
        int windowSize;
        simtime_t routeTimeout;
        bool storeBestRoutesOnly;
        bool getRoutesFromDataPackets;
        simtime_t stopRoutingAfterDataDone;

        double routingPacketPriority;
        double ownDataPriority;
        int packetTTL;

        //Node info
        int nodeId;

        std::vector<int> neighbourNodes;
        std::vector<int> knownNodes;
        std::vector<int> ACKedNodes;
        std::vector<LoRaAppPacket> LoRaPacketsToSend;
        std::vector<LoRaAppPacket> LoRaPacketsToForward;
        std::vector<LoRaAppPacket> LoRaPacketsForwarded;
        std::vector<LoRaAppPacket> DataPacketsForMe;

        //Application parameters
        bool requestACKfromApp;
        bool stopOnACK;
        bool AppACKReceived;
        int firstACK;

        //Spreading factor
        bool increaseSF;
        int firstACKSF;
        int packetsPerSF;
        int packetsInSF;

        //LoRa settings
        int minLoRaSF;
        int maxLoRaSF;

        //Forwarded packets vector size
        int forwardedPacketVectorSize;

        //Forward packets buffer max vector size
        int packetsToForwardMaxVectorSize;

        // Routing tables
        class singleMetricRoute {

            public:
                int id;
                int via;
                double metric;
                int window[33];
                simtime_t valid;
        };
        std::vector<singleMetricRoute> singleMetricRoutingTable;

        class dualMetricRoute {

            public:
                int id;
                int via;
                double priMetric;
                double secMetric;
                int window[33];
                int sf;
                simtime_t valid;
        };
        std::vector<dualMetricRoute> dualMetricRoutingTable;


        /**
         * @name CsmaCaMac state variables
         * Various state information checked and modified according to the state machine.
         */
        //@{
        enum State {
            IDLE,
            TRANSMIT,
            WAIT_DELAY_1,
            LISTENING_1,
            RECEIVING_1,
            WAIT_DELAY_2,
            LISTENING_2,
            RECEIVING_2,
        };

        //LoRa parameters control
        LoRaRadio *loRaRadio;

        void setSF(int SF);
        int getSF();
        void setTP(int TP);
        double getTP();
        void setCR(int CR);
        int getCR();
        void setCF(units::values::Hz CF);
        units::values::Hz getCF();
        void setBW(units::values::Hz BW);
        units::values::Hz getBW();

        void setLoRaTagToPkt(Packet *packet, int customSF=-1);

        static cEnum *appModeEnum;

        simtime_t switchingTimes[APP_MODE_SWITCHING][APP_MODE_SWITCHING];
        AppMode appMode, nextAppMode, previousAppMode;
        cMessage *selfAppModeSwitchTimerMsg;

    private:
        void parseAppModeSwitchingTimes();
        void startAppModeSwitch(AppMode newAppMode, simtime_t switchingTime);
        void completeAppModeSwitch(AppMode newAppMode);
        void handlePacketTxSelfMessage(cMessage *msg);
        void handleTaskTimerSelfMessage(cMessage *msg);
        void handleAppModeSwitchTimerSelfMessage(cMessage *msg);
        void cleanTeardown(void);
};

}

#endif
