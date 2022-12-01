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

#include "LoRaNodeApp.h"
#include "inet/common/FSMA.h"
#include "../LoRa/LoRaMac.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/mobility/static/StationaryMobility.h"
#include <cmath>

namespace flora {

#define BROADCAST_ADDRESS   16777215

#define NO_FORWARDING                    0  // No forwarding, no routing.
#define FLOODING_BROADCAST_SINGLE_SF     1  // Forwarding by flooding, no routing.
#define SMART_BROADCAST_SINGLE_SF        2  // Forwarding by flooding, no routing, single-hop unicast for last hop if neighbour known.
#define HOP_COUNT_SINGLE_SF              3  // Next hop routing based on single-SF hop count. In case of a tie, select newest route first.
#define RSSI_SUM_SINGLE_SF               4  // Next hop routing based on single-SF cumulative RSSI. In case of a tie, select newest route first.
#define RSSI_PROD_SINGLE_SF              5  // Next hop routing based on single-SF multiplicative RSSI. In case of a tie, select newest route first.
#define ETX_SINGLE_SF                    6  // Next hop routing based on single-SF ETX. In case of a tie, select newest route first.
#define TIME_ON_AIR_NEWEST_CAD_MULTI_SF 10  // Next hop routing based on multi-SF ToA. In case of a tie, select newest route first.
#define TIME_ON_AIR_RANDOM_CAD_MULTI_SF 11  // Next hop routing based on multi-SF ToA. In case of a tie, select randomly between drawing routes.
#define TIME_ON_AIR_HC_CAD_MULTI_SF     12  // Next hop routing based on multi-SF ToA. In case of a tie, select by minimum hop count between drawing routes. In case of a tie, select newest route first.
#define TIME_ON_AIR_ETX_CAD_MULTI_SF    13  // Next hop routing based on multi-SF ETX-weighted ToA. In case of a tie, select newest route first.
#define TIME_ON_AIR_FQUEUE_CAD_MULTI_SF 14  // Next hop routing based on multi-SF queue-length-weighted ToA. In case of a tie, select newest route first.
#define TIME_ON_AIR_RMP1_CAD_MULTI_SF   15  // Next hop routing based on multi-SF ToA. In case of a tie, select newest route first. Randomly use higher SFs than required.

Define_Module(LoRaNodeApp);

simsignal_t LoRaNodeApp::appModeChangedSignal = cComponent::registerSignal("appModeChanged");
simsignal_t LoRaNodeApp::commActiveChangedSignal = cComponent::registerSignal("commActiveChanged");
cEnum *LoRaNodeApp::appModeEnum = nullptr;

Register_Enum(LoRaNodeApp::AppMode,
    (LoRaNodeApp::APP_MODE_SLEEP,
     LoRaNodeApp::APP_MODE_RUN,
     LoRaNodeApp::APP_MODE_SWITCHING));

void LoRaNodeApp::initialize(int stage) {
    cSimpleModule::initialize(stage);

    //Current network settings
    numberOfNodes = par("numberOfNodes");

    if (stage == INITSTAGE_LOCAL) {
        // Get this node's ID
        nodeId = getContainingNode(this)->getIndex();
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        if (strcmp(host->par("deploymentType").stringValue(), "manual") == 0) {} else
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle") == 0) {
            coordsValues = generateUniformCircleCoordinates(
                    host->par("maxGatewayDistance").doubleValue(),
                    host->par("gatewayX").doubleValue(),
                    host->par("gatewayY").doubleValue());
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(coordsValues.first);
            mobility->par("initialY").setDoubleValue(coordsValues.second);
        } else if (strcmp(host->par("deploymentType").stringValue(), "edges")
                == 0) {
            int minX = (int)host->par("minX").doubleValue();
            int maxX = (int)host->par("maxX").doubleValue();
            int minY = (int)host->par("minY").doubleValue();
            int maxY = (int)host->par("maxY").doubleValue();
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(
                    minX + maxX * (((nodeId + 1) % 4 / 2) % 2));
            mobility->par("initialY").setDoubleValue(
                    minY + maxY * (((nodeId) % 4 / 2) % 2));
        } else if (strcmp(host->par("deploymentType").stringValue(), "grid") == 0) {
            int minX = (int)host->par("minX").doubleValue();
            int sepX = (int)host->par("sepX").doubleValue();
            int minY = (int)host->par("minY").doubleValue();
            int sepY = (int)host->par("sepY").doubleValue();
            int cols = int(sqrt(numberOfNodes));
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(
                    minX + sepX * (nodeId % cols));
            mobility->par("initialY").setDoubleValue(
                    minY + sepY * ((int) nodeId / cols));
        } else {
            int minX = (int)host->par("minX").doubleValue();
            int maxX = (int)host->par("maxX").doubleValue();
            int minY = (int)host->par("minY").doubleValue();
            int maxY = (int)host->par("maxY").doubleValue();
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(uniform(minX, maxX));
            mobility->par("initialY").setDoubleValue(uniform(minY, maxY));
        }
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;

        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        if (!isOperational)
        {
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        }

        initializeAppMode();
        parseAppModeSwitchingTimes();

        // Initialize counters
        sentPackets = 0;
        sentDataPackets = 0;
        sentRoutingPackets = 0;
        sentAckPackets = 0;
        receivedPackets = 0;
        receivedPacketsForMe = 0;
        receivedPacketsFromMe = 0;
        receivedPacketsToForward = 0;
        receivedRoutingPackets = 0;
        receivedDataPackets = 0;
        receivedDataPacketsNotForMe = 0;
        receivedDataPacketsForMe = 0;
        receivedDataPacketsForMeUnique = 0;
        receivedDataPacketsFromMe = 0;
        receivedDataPacketsToForward = 0;
        receivedDataPacketsToForwardCorrect = 0;
        receivedDataPacketsToForwardExpired = 0;
        receivedDataPacketsToForwardUnique = 0;
        receivedAckPackets = 0;
        receivedAckPacketsForMe = 0;
        receivedAckPacketsFromMe = 0;
        receivedAckPacketsToForward = 0;
        receivedAckPacketsToForwardCorrect = 0;
        receivedAckPacketsToForwardExpired = 0;
        receivedAckPacketsToForwardUnique = 0;
        receivedADRCommands = 0;
        forwardedPackets = 0;
        forwardedDataPackets = 0;
        forwardedAckPackets = 0;
        forwardPacketsDuplicateAvoid = 0;
        packetsToForwardMaxVectorSize = 0;
        broadcastDataPackets = 0;
        broadcastForwardedPackets = 0;
        deletedRoutes = 0;
        forwardBufferFull = 0;

        firstDataPacketTransmissionTime = 0;
        lastDataPacketTransmissionTime = 0;
        firstDataPacketReceptionTime = 0;
        lastDataPacketReceptionTime = 0;

        averageUpdateSensorWeight = par("averageUpdateSensorWeight");

        averageTemp = par("iniTemperature");
        tempFireThreshold = par("temperatureFireThreshold");
        tempSensor = new TempSensorNode(averageTemp);

        averageHumidity = par("iniHumidity");
        humidityFireThreshold = par("humidityFireThreshold");
        humiditySensor = new HumiditySensorNode(averageHumidity);

        timeToForceFireStart = simTime() + par("timeToForceFireStart");
        forceFireSensors = par("forceFireSensors");
        forceFireCondition = par("forceFireCondition");

        fireAlarmGatewayNodeId = par("fireAlarmGatewayNodeId");
        fireAlarmEnable = par("fireAlarmEnable");
        fireAlarmOnce = par("fireAlarmOnce");
        fireAlarmTriggered = false;

        dataPacketsDue = false;
        routingPacketsDue = false;

        sendPacketsContinuously = par("sendPacketsContinuously");
        onlyNode0SendsPackets = par("onlyNode0SendsPackets");
        enforceDutyCycle = par("enforceDutyCycle");
        dutyCycle = par("dutyCycle");
        numberOfDestinationsPerNode = par("numberOfDestinationsPerNode");
        numberOfPacketsPerDestination = par("numberOfPacketsPerDestination");

        numberOfPacketsToForward = par("numberOfPacketsToForward");

        packetsToForwardMaxVectorSize = par("packetsToForwardMaxVectorSize");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        currDataInt = 0;

        //LoRa physical layer parameters
        loRaRadio = check_and_cast<LoRaRadio *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("radio"));
        loRaRadio->loRaTP = par("initialLoRaTP").doubleValue();
        loRaRadio->loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaRadio->loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaRadio->loRaCR = par("initialLoRaCR");
        loRaRadio->loRaUseHeader = par("initialUseHeader");

        minLoRaSF = par("minLoRaSF");
        maxLoRaSF = par("maxLoRaSF");
        loRaRadio->loRaSF = par("initialLoRaSF");
        if (loRaRadio->loRaSF < minLoRaSF) {
            loRaRadio->loRaSF = minLoRaSF;
        } else if (loRaRadio->loRaSF > maxLoRaSF) {
            loRaRadio->loRaSF = maxLoRaSF;
        }

        loRaRadio->loRaCAD = par("initialLoRaCAD");
        loRaRadio->loRaCADatt = par("initialLoRaCADatt").doubleValue();

        evaluateADRinNode = par("evaluateADRinNode");

        txSfVector.setName("Tx SF Vector");
        txTpVector.setName("Tx TP Vector");
        rxRssiVector.setName("Rx RSSI Vector");
        rxSfVector.setName("Rx SF Vector");

        //Routing variables
        routingMetric = par("routingMetric");
        routeDiscovery = par("routeDiscovery");
        // Route discovery must be enabled for broadcast-based smart forwarding
        switch (routingMetric) {
            case SMART_BROADCAST_SINGLE_SF:
                routeDiscovery = true;
        }
        routingPacketPriority = par("routingPacketPriority");
        ownDataPriority = par("ownDataPriority");
        routeTimeout = par("routeTimeout");
        storeBestRoutesOnly = par("storeBestRouteOnly");
        getRoutesFromDataPackets = par("getRoutesFromDataPackets");
        packetTTL = par("packetTTL");
        stopRoutingAfterDataDone = par("stopRoutingAfterDataDone");

        windowSize = std::min(32,(int)math::maxnan(1.0,(double)par("windowSize").intValue())); //Must be an int between 1 and 32

        if ( packetTTL == 0 ) {
            if (strcmp(getContainingNode(this)->par("deploymentType").stringValue(), "grid") == 0) {
                packetTTL = 2*(sqrt(numberOfNodes)-1);
                if (routingMetric != 0) {
                    packetTTL = math::maxnan(2.0,2*(sqrt(numberOfNodes)-1));
                }
            }
            else {
                packetTTL = 2*(sqrt(numberOfNodes));
                if (routingMetric != 0) {
                    packetTTL = math::maxnan(2.0,2*(sqrt(numberOfNodes)-1));
                }
            }
        }

        //Packet sizes
        dataPacketSize = par("dataPacketDefaultSize");
        routingPacketMaxSize = par("routingPacketMaxSize");

        // Routing packets timing
        timeToNextRoutingPacketMin = par("timeToNextRoutingPacketMin");
        timeToNextRoutingPacketMax = par("timeToNextRoutingPacketMax");
        timeToNextRoutingPacketAvg = par("timeToNextRoutingPacketAvg");

        simTimeResolution = pow(10, simTimeResolution.getScaleExp());

        neighbourNodes = {};
        knownNodes = {};
        LoRaPacketsToSend = {};
        LoRaPacketsToForward = {};
        LoRaPacketsForwarded = {};
        DataPacketsForMe = {};
        ACKedNodes = {};

        //Routing table
        singleMetricRoutingTable = {};
        dualMetricRoutingTable = {};

        //Node identifier
        nodeId = getContainingNode(this)->getIndex();

        //Application acknowledgment
        requestACKfromApp = par("requestACKfromApp");
        stopOnACK = par("stopOnACK");
        AppACKReceived = false;
        firstACK = 0;

        //Spreading factor
        increaseSF = par("increaseSF");
        firstACKSF = 0;
        packetsPerSF = par("packetsPerSF");
        packetsInSF = 0;

        // Communication Active Window
        commActiveState = false;
        selfCommTimerMsg = new cMessage("selfCommTimerMsg");
        commActivePeriod = par("commActivePeriod");
        commActiveDutyCycle = par("commActiveDutyCycle");
        commActiveTxRatio = par("commActiveTxRatio");
        cachedDataPktToSendInfo = std::make_pair(nullptr, 0);

        double commActiveWindow = commActivePeriod.dbl()*commActiveDutyCycle;
        timeToNextCommActivePeriodStart = simTime() + 10*simTimeResolution;
        timeToNextCommActiveRxOnly = timeToNextCommActivePeriodStart + commActiveTxRatio*commActiveWindow;
        timeToNextCommActivePeriodEnd = timeToNextCommActivePeriodStart + commActiveWindow;
        scheduleAt(timeToNextCommActivePeriodStart, selfCommTimerMsg);

        //Forwarded packets vector size
        forwardedPacketVectorSize = par("forwardedPacketVectorSize");

        //WATCHES only for GUI
        if (getEnvir()->isGUI()) {
            WATCH(sentPackets);
            WATCH(sentDataPackets);
            WATCH(sentRoutingPackets);
            WATCH(sentAckPackets);
            WATCH(receivedPackets);
            WATCH(receivedPacketsForMe);
            WATCH(receivedPacketsFromMe);
            WATCH(receivedPacketsToForward);
            WATCH(receivedRoutingPackets);
            WATCH(receivedDataPackets);
            WATCH(receivedDataPacketsNotForMe);
            WATCH(receivedDataPacketsForMe);
            WATCH(receivedDataPacketsForMeUnique);
            WATCH(receivedDataPacketsFromMe);
            WATCH(receivedDataPacketsToForward);
            WATCH(receivedDataPacketsToForwardCorrect);
            WATCH(receivedDataPacketsToForwardExpired);
            WATCH(receivedDataPacketsToForwardUnique);
            WATCH(receivedAckPackets);
            WATCH(receivedAckPacketsForMe);
            WATCH(receivedAckPacketsFromMe);
            WATCH(receivedAckPacketsToForward);
            WATCH(receivedAckPacketsToForwardCorrect);
            WATCH(receivedAckPacketsToForwardExpired);
            WATCH(receivedAckPacketsToForwardUnique);
            WATCH(receivedADRCommands);
            WATCH(forwardedPackets);
            WATCH(forwardedDataPackets);
            WATCH(forwardedAckPackets);
            WATCH(forwardPacketsDuplicateAvoid);
            WATCH(packetsToForwardMaxVectorSize);
            WATCH(broadcastDataPackets);
            WATCH(broadcastForwardedPackets);
            WATCH(deletedRoutes);
            WATCH(forwardBufferFull);

            WATCH(AppACKReceived);
            WATCH(firstACK);
            WATCH(packetTTL);
            WATCH(loRaRadio->loRaSF);
            WATCH(packetsInSF);

            WATCH_VECTOR(neighbourNodes);
            WATCH_VECTOR(knownNodes);
            WATCH_VECTOR(ACKedNodes);

            WATCH(firstDataPacketTransmissionTime);
            WATCH(lastDataPacketTransmissionTime);
            WATCH(firstDataPacketReceptionTime);
            WATCH(lastDataPacketReceptionTime);

            //WATCH_VECTOR(singleMetricRoutingTable);
            //WATCH_VECTOR(dualMetricRoutingTable);

            WATCH_VECTOR(LoRaPacketsToSend);
            WATCH_VECTOR(LoRaPacketsToForward);
            WATCH_VECTOR(LoRaPacketsForwarded);
            WATCH_VECTOR(DataPacketsForMe);
            WATCH(appMode);
        }

        if (numberOfDestinationsPerNode == 0 ) {
            numberOfDestinationsPerNode = numberOfNodes-1;
        }

        selfPacketTxMsg = new cMessage("selfPacketTxMsg");
        selfTaskTimerMsg = new cMessage("selfTaskTimerMsg");
        selfAppModeSwitchTimerMsg = new cMessage("selfAppModeSwitchTimerMsg");

        // Routing packets timer
        timeToFirstRoutingPacket = math::maxnan(5.0, (double)par("timeToFirstRoutingPacket"))+getTimeToNextRoutingPacket();
        switch (routingMetric) {
            // No routing packets are to be sent
            case NO_FORWARDING:
            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
                break;
            // Schedule selfRoutingPackets
            default:
                routingPacketsDue = true;
                nextRoutingPacketTransmissionTime = timeToFirstRoutingPacket;
                EV << "Time to first routing packet: " << timeToFirstRoutingPacket << endl;
                break;
        }

        if (routingPacketsDue) {
            scheduleAt(simTime() + timeToFirstRoutingPacket, selfPacketTxMsg);
            EV << "Self Packet Tx Msg triggered by due routing packet" << endl;
        }

        // Task timer
        timeToFirstTaskTimerTick = math::maxnan(5.0, (double)par("timeToFirstTaskTimerTick"));
        timeToNextTaskTimerTick = par("timeToNextTaskTimerTick");
        scheduleAt(simTime() + timeToFirstTaskTimerTick, selfTaskTimerMsg);
    }
}

std::pair<double, double> LoRaNodeApp::generateUniformCircleCoordinates(
    double radius, double gatewayX, double gatewayY) {
    double randomValueRadius = uniform(0, (radius * radius));
    double randomTheta = uniform(0, 2 * M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double, double> coordValues = std::make_pair(x, y);
    return coordValues;
}

void LoRaNodeApp::initializeAppMode() {
    const char *initialAppMode = par("initialAppMode");
    if (!strcmp(initialAppMode, "sleep"))
        completeAppModeSwitch(LoRaNodeApp::APP_MODE_SLEEP);
    else if (!strcmp(initialAppMode, "run"))
        completeAppModeSwitch(LoRaNodeApp::APP_MODE_RUN);
    else
        throw cRuntimeError("Unknown initialAppMode");
}

void LoRaNodeApp::parseAppModeSwitchingTimes()
{
    const char *times = par("switchingTimes");

    char prefix[3];
    unsigned int count = sscanf(times, "%s", prefix);

    if (count > 2)
        throw cRuntimeError("Metric prefix should be no more than two characters long");

    double metric = 1;

    if (strcmp("s", prefix) == 0)
        metric = 1;
    else if (strcmp("ms", prefix) == 0)
        metric = 0.001;
    else if (strcmp("ns", prefix) == 0)
        metric = 0.000000001;
    else
        throw cRuntimeError("Undefined or missed metric prefix for switchingTimes parameter");

    cStringTokenizer tok(times + count + 1);
    unsigned int idx = 0;
    while (tok.hasMoreTokens()) {
        switchingTimes[idx / APP_MODE_SWITCHING][idx % APP_MODE_SWITCHING] = atof(tok.nextToken()) * metric;
        idx++;
    }
    if (idx != APP_MODE_SWITCHING * APP_MODE_SWITCHING)
        throw cRuntimeError("Check your switchingTimes parameter! Some parameters may be missed");
}

void LoRaNodeApp::setAppMode(AppMode newAppMode)
{
    Enter_Method("setAppMode");
    if (newAppMode < APP_MODE_SLEEP || newAppMode > APP_MODE_SWITCHING)
        throw cRuntimeError("Unknown app mode: %d", newAppMode);
    else if (newAppMode == APP_MODE_SWITCHING)
        throw cRuntimeError("Cannot switch manually to APP_MODE_SWITCHING");
    else if (appMode == APP_MODE_SWITCHING || selfAppModeSwitchTimerMsg->isScheduled())
        throw cRuntimeError("Cannot switch to a new APP mode while another switch is in progress");
    else if (newAppMode != appMode && newAppMode != nextAppMode) {
        simtime_t switchingTime = switchingTimes[appMode][newAppMode];
        if (switchingTime != 0)
            startAppModeSwitch(newAppMode, switchingTime);
        else
            completeAppModeSwitch(newAppMode);
    }
}

void LoRaNodeApp::startAppModeSwitch(AppMode newAppMode, simtime_t switchingTime)
{
    //EV_DETAIL << "Starting to change App mode from \x1b[1m" << appMode << "\x1b[0m to \x1b[1m" << newAppMode << "\x1b[0m." << endl;
    previousAppMode = appMode;
    appMode = APP_MODE_SWITCHING;
    nextAppMode = newAppMode;
    emit(appModeChangedSignal, appMode);
    scheduleAfter(switchingTime, selfAppModeSwitchTimerMsg);
}

void LoRaNodeApp::completeAppModeSwitch(AppMode newAppMode)
{
    //EV_INFO << "App mode changed from \x1b[1m" << previousAppMode << "\x1b[0m to \x1b[1m" << newAppMode << "\x1b[0m." << endl;
    appMode = previousAppMode = nextAppMode = newAppMode;
    emit(appModeChangedSignal, newAppMode);
}

void LoRaNodeApp::cleanTeardown()
{
    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToSend.begin();
            lbptr < LoRaPacketsToSend.end(); lbptr++) {
        LoRaPacketsToSend.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        LoRaPacketsToForward.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        LoRaPacketsForwarded.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        DataPacketsForMe.erase(lbptr);
    }
    cancelEvent(selfTaskTimerMsg);
}

void LoRaNodeApp::finish() {
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
            host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaRadio->loRaTP);
    recordScalar("finalSF", loRaRadio->loRaSF);

    recordScalar("sentPackets", sentPackets);
    recordScalar("sentDataPackets", sentDataPackets);
    recordScalar("sentRoutingPackets", sentRoutingPackets);
    recordScalar("sentAckPackets", sentAckPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedPacketsForMe", receivedPacketsForMe);
    recordScalar("receivedPacketsFromMe", receivedPacketsFromMe);
    recordScalar("receivedPacketsToForward", receivedPacketsToForward);
    recordScalar("receivedDataPackets", receivedDataPackets);
    recordScalar("receivedDataPacketsNotForMe", receivedDataPacketsNotForMe);
    recordScalar("receivedDataPacketsForMe", receivedDataPacketsForMe);
    recordScalar("receivedDataPacketsForMeUnique", receivedDataPacketsForMeUnique);
    recordScalar("receivedDataPacketsFromMe", receivedDataPacketsFromMe);
    recordScalar("receivedDataPacketsToForward", receivedDataPacketsToForward);
    recordScalar("receivedDataPacketsToForwardCorrect",
            receivedDataPacketsToForwardCorrect);
    recordScalar("receivedDataPacketsToForwardExpired",
            receivedDataPacketsToForwardExpired);
    recordScalar("receivedDataPacketsToForwardUnique",
            receivedDataPacketsToForwardUnique);
    recordScalar("receivedAckPacketsToForward", receivedAckPacketsToForward);
    recordScalar("receivedAckPacketsToForwardCorrect",
            receivedAckPacketsToForwardCorrect);
    recordScalar("receivedAckPacketsToForwardExpired",
            receivedAckPacketsToForwardExpired);
    recordScalar("receivedAckPacketsToForwardUnique",
            receivedAckPacketsToForwardUnique);
    recordScalar("receivedAckPackets", receivedAckPackets);
    recordScalar("receivedAckPacketsForMe", receivedAckPacketsForMe);
    recordScalar("receivedAckPacketsFromMe", receivedAckPacketsFromMe);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("forwardedPackets", forwardedPackets);
    recordScalar("forwardedDataPackets", forwardedDataPackets);
    recordScalar("forwardedAckPackets", forwardedAckPackets);
    recordScalar("forwardPacketsDuplicateAvoid", forwardPacketsDuplicateAvoid);
    recordScalar("packetsToForwardMaxVectorSize", packetsToForwardMaxVectorSize);
    recordScalar("broadcastDataPackets", broadcastDataPackets);
    recordScalar("broadcastForwardedPackets", broadcastForwardedPackets);

    recordScalar("firstDataPacketTransmissionTime", firstDataPacketTransmissionTime);
    recordScalar("lastDataPacketTransmissionTime", lastDataPacketTransmissionTime);
    recordScalar("firstDataPacketReceptionTime", firstDataPacketReceptionTime);
    recordScalar("lastDataPacketReceptionTime", lastDataPacketReceptionTime);

    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("AppACKReceived", AppACKReceived);
    recordScalar("firstACK", firstACK);
    recordScalar("firstACKSF", firstACKSF);

    recordScalar("dataPacketsNotSent", LoRaPacketsToSend.size());
    recordScalar("forwardPacketsNotSent", LoRaPacketsToForward.size());

    recordScalar("forwardBufferFull", forwardBufferFull);

    recordScalar("dataPacketsForMeLatencyMax", dataPacketsForMeLatency.getMax());
    recordScalar("dataPacketsForMeLatencyMean", dataPacketsForMeLatency.getMean());
    recordScalar("dataPacketsForMeLatencyMin", dataPacketsForMeLatency.getMin());
    recordScalar("dataPacketsForMeLatencyStdv", dataPacketsForMeLatency.getStddev());

    recordScalar("dataPacketsForMeUniqueLatencyMax", dataPacketsForMeUniqueLatency.getMax());
    recordScalar("dataPacketsForMeUniqueLatencyMean", dataPacketsForMeUniqueLatency.getMean());
    recordScalar("dataPacketsForMeUniqueLatencyMin", dataPacketsForMeUniqueLatency.getMin());
    recordScalar("dataPacketsForMeUniqueLatencyStdv", dataPacketsForMeUniqueLatency.getStddev());

    recordScalar("routingTableSizeMax", routingTableSize.getMax());
    recordScalar("routingTableSizeMean", routingTableSize.getMean());
    recordScalar("routingTableSizeMin", routingTableSize.getMin());
    recordScalar("routingTableSizeStdv", routingTableSize.getStddev());

    recordScalar("allTxPacketsSFStatsMax", allTxPacketsSFStats.getMax());
    recordScalar("allTxPacketsSFStatsMean", allTxPacketsSFStats.getMean());
    recordScalar("allTxPacketsSFStatsMin", allTxPacketsSFStats.getMin());
    recordScalar("allTxPacketsSFStatsStdv", allTxPacketsSFStats.getStddev());
    recordScalar("routingTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("routingTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("routingTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("routingTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());
    recordScalar("owndataTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("owndataTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("owndataTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("owndataTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());
    recordScalar("fwdTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("fwdTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("fwdTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("fwdTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());

    dataPacketsForMeLatency.recordAs("dataPacketsForMeLatency");
    dataPacketsForMeUniqueLatency.recordAs("dataPacketsForMeUniqueLatency");

    cleanTeardown();
}

void LoRaNodeApp::handlePacketTxSelfMessage(cMessage *msg) {
    // Only proceed to send a data packet if the 'mac' module in 'LoRaNic' is ACTIVE and the warmup period is due
    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");
    bool sendSuccess = false;

    if (lrmc->fsm.getState() == ACTIVE) {
        bool sendRouting = false;
        bool sendData = false;

        // Since the MAC is active we must have timeToNextCommActivePeriodStart < simTime() < timeToNextCommActivePeriodEnd
        if (!(simTime() >= timeToNextCommActivePeriodStart && simTime() < timeToNextCommActivePeriodEnd)) {
            throw cRuntimeError("Attempting Tx outside communication active window");
        }

        // Check if there are routing packets to send, and if it is time to send them
        if (routingPacketsDue && simTime() >= nextRoutingPacketTransmissionTime) {
            sendRouting = true;
        }
        // Check if there are data packets to send or forward, and if it is time to send them
        if ((LoRaPacketsToSend.size() > 0 || LoRaPacketsToForward.size() > 0 ) && simTime() >= nextDataPacketTransmissionTime) {
            sendData = true;
        }

        // If both types of packets are pending, decide which one will be sent
        if (sendRouting && sendData) {
            // Either send a routing packet...
            if (bernoulli(routingPacketPriority)) {
                sendData = false;
            } else {
                sendRouting = false;
            }
        }

        // Send routing packet
        if (sendRouting) {
            auto pktInfo = getRoutingPacketToSend();
            auto pkt = pktInfo.first;
            if (pkt != nullptr) {
                auto txDuration = pktInfo.second;
                // Make sure that the Tx duration fits in the comm active Tx window, if not then reschedule
                bool txInWindow = (simTime() + txDuration) <  timeToNextCommActiveRxOnly;
                if (txInWindow) {
                    sendRoutingPacket(pkt);

                    // Calculate next transmission time after sending
                    minNextPacketTransmissionTime = simTime() + (enforceDutyCycle ? txDuration/dutyCycle : txDuration);
                    nextRoutingPacketTransmissionTime = math::maxnan(simTime().dbl() + getTimeToNextRoutingPacket().dbl(), minNextPacketTransmissionTime.dbl());

                    sendSuccess = true;
                } else {
                    // Cache packet info for later transmission
                    EV_INFO << "Tx not in window for routing packet, caching..." << endl;
                    cachedRoutingPktToSendInfo = pktInfo;
                }
            }
        }
        // Send or forward data packet
        else if (sendData) {
            auto pktInfo = getDataPacketToSend();
            auto pkt = pktInfo.first;
            if (pkt != nullptr) {
                auto txDuration = pktInfo.second;
                // Make sure that the Tx duration fits in the comm active Tx window, if not then reschedule
                bool txInWindow = (simTime() + txDuration) <  timeToNextCommActiveRxOnly;
                if (txInWindow) {
                    sendDataPacket(pkt);

                    // Calculate next transmission time after sending
                    minNextPacketTransmissionTime = simTime() + (enforceDutyCycle ? txDuration/dutyCycle : txDuration);
                    nextDataPacketTransmissionTime = minNextPacketTransmissionTime;

                    if (LoRaPacketsToSend.size() > 0 || LoRaPacketsToForward.size() > 0) {
                        dataPacketsDue = true;
                    } else {
                        dataPacketsDue = false;
                    }
                    sendSuccess = true;
                } else {
                    // Cache packet info for later transmission
                    EV_INFO << "Tx not in window for local or forward data packet, caching..." << endl;
                    cachedDataPktToSendInfo = pktInfo;
                }
            }
        }
    }

    if (sendSuccess) {
        simtime_t nextScheduleTime = 0;

        if (routingPacketsDue && dataPacketsDue) {
            nextScheduleTime = std::min(nextRoutingPacketTransmissionTime.dbl(), nextDataPacketTransmissionTime.dbl());
        } else if (routingPacketsDue) {
            nextScheduleTime = nextRoutingPacketTransmissionTime;
        } else if (dataPacketsDue) {
            nextScheduleTime = nextDataPacketTransmissionTime;
        }
        nextScheduleTime = math::maxnan(nextScheduleTime.dbl(), minNextPacketTransmissionTime.dbl());

        // Last, check the schedule time is in the future, otherwise just add a 1s delay
        if (!(nextScheduleTime > simTime())) {
            nextScheduleTime = simTime() + 1;
        }

        // Schedule a self message to send routing or data packets. Add a
        // simulation time delta (i.e., a simtime-resolution unit) to avoid
        // timing conflicts in the LoRaMac layer
        if (routingPacketsDue || dataPacketsDue) {
            scheduleAt(nextScheduleTime + 10*simTimeResolution, selfPacketTxMsg);
        }
    } else {
        EV_INFO << "Scheduling Tx for start of next active period" << endl;
        simtime_t nextScheduleTime = 0;
        if (simTime() > timeToNextCommActivePeriodStart) {
            nextScheduleTime = timeToNextCommActivePeriodStart + commActivePeriod;
        } else {
            nextScheduleTime = timeToNextCommActivePeriodStart;
        }
        scheduleAt(nextScheduleTime + 10*simTimeResolution, selfPacketTxMsg);
    }
}

void LoRaNodeApp::handleTaskTimerSelfMessage(cMessage *msg) {
    setAppMode(APP_MODE_RUN);

    double currentTemp = tempSensor->getData();
    double currentHumidity = humiditySensor->getData();
    double ausw = averageUpdateSensorWeight;
    bool isForceFireTime = simTime() > timeToForceFireStart;

    if (forceFireSensors && isForceFireTime) {
        currentTemp = tempSensor->forceFire();
        currentHumidity = tempSensor->forceFire();
        forceFireSensors = false;
    }

    averageTemp = ausw * currentTemp + (1-ausw) * averageTemp;
    averageHumidity = ausw * currentHumidity + (1-ausw) * averageHumidity;

    // EV_INFO << "Average temp: " << averageTemp << " / threshold: " << tempFireThreshold
    //         << "Average humidity: " << averageHumidity << " / threshold: " << humidityFireThreshold
    //         << endl;

    // TODO: Humidity threshold should be lower than (also fix HumiditySensor fire condition)
    bool fireAlarmCondition = fireAlarmEnable && ((fireAlarmOnce && !fireAlarmTriggered) || !fireAlarmOnce);
    bool fireCondition = (averageTemp > tempFireThreshold) && (averageHumidity > humidityFireThreshold);

    if (fireAlarmCondition && (fireCondition || (forceFireCondition && isForceFireTime))) {
        EV_INFO << "Triggering fire alarm!" << endl;
        fireAlarmTriggered = true;
        generateFireAlarmPacket();
        simtime_t nextScheduleTime = math::maxnan(simTime().dbl(), minNextPacketTransmissionTime.dbl()) + 10*simTimeResolution;
        rescheduleAt(nextScheduleTime, selfPacketTxMsg);
    }

    // Go to sleep and schedule next task timer tick
    setAppMode(APP_MODE_SLEEP);
    scheduleAt(simTime() + timeToNextTaskTimerTick, selfTaskTimerMsg);
}

void LoRaNodeApp::handleAppModeSwitchTimerSelfMessage(cMessage *msg) {
    completeAppModeSwitch(nextAppMode);
}

void LoRaNodeApp::handleCommTimerSelfMessage(cMessage *msg) {

    if (!commActiveState) {
        // Start window and schedule end
        scheduleAt(timeToNextCommActivePeriodEnd, selfCommTimerMsg);
        setCommActiveState(true);
    } else {
        // End window and schedule start
        double commActiveWindow = commActivePeriod.dbl()*commActiveDutyCycle;
        timeToNextCommActivePeriodStart = simTime() + (1-commActiveDutyCycle) * commActivePeriod;
        timeToNextCommActiveRxOnly = timeToNextCommActivePeriodStart + commActiveTxRatio*commActiveWindow;
        timeToNextCommActivePeriodEnd = timeToNextCommActivePeriodStart + commActiveWindow;
        scheduleAt(timeToNextCommActivePeriodStart, selfCommTimerMsg);
        setCommActiveState(false);
    }
}

void LoRaNodeApp::handleSelfMessage(cMessage *msg) {

    if (msg == selfPacketTxMsg) {
        handlePacketTxSelfMessage(msg);
    } else if (msg == selfTaskTimerMsg) {
        handleTaskTimerSelfMessage(msg);
    } else if (msg == selfAppModeSwitchTimerMsg) {
        handleAppModeSwitchTimerSelfMessage(msg);
    } else if (msg == selfCommTimerMsg){
        handleCommTimerSelfMessage(msg);
    } else {
        throw cRuntimeError("Unknown self message");
    }
}

void LoRaNodeApp::generateFireAlarmPacket() {
    auto dataPacket = makeShared<LoRaAppPacket>();

    dataPacket->setMsgType(DATA);
    dataPacket->setDataInt(currDataInt++);
    // dataPacket->setDataInt(averageTemp);
    dataPacket->setSource(nodeId);
    dataPacket->setVia(nodeId);

    dataPacket->setDestination(fireAlarmGatewayNodeId);
    LoRaOptions opts = dataPacket->getOptions();
    opts.setAppACKReq(requestACKfromApp);
    dataPacket->setOptions(opts);
    dataPacket->setChunkLength(B(dataPacketSize));
    dataPacket->setDepartureTime(simTime());
    dataPacket->setTtl(packetTTL);

    LoRaPacketsToSend.clear();
    LoRaPacketsToSend.push_back(*dataPacket);
}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    receivedPackets++;

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    // Check if the packet is from this node (i.e., a packet that some
    // other node is broadcasting which we have happened to receive). We
    // count it and discard it immediately.
    if (packet->getSource() == nodeId) {
        receivedDataPacketsFromMe++;
        bubble("I received a LoRa packet originally sent by me!");
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else, check if the packet is for this node (i.e., a packet directly
    // received from the origin or relayed by a neighbour)
    else if (packet->getDestination() == nodeId) {
        bubble("I received a data packet for me!");
        manageReceivedPacketForMe(msg);
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else it can be a routing protocol broadcast message
    else if (packet->getDestination() == BROADCAST_ADDRESS) {
        manageReceivedRoutingPacket(msg);
    }
    // Else it can be a data packet from and to other nodes...
    else {
        // which we may forward, if it is being broadcast
        if (packet->getVia() == BROADCAST_ADDRESS && routeDiscovery == true) {
            bubble("I received a multicast data packet to forward!");
            manageReceivedDataPacketToForward(msg);
            if (firstDataPacketReceptionTime == 0) {
                firstDataPacketReceptionTime = simTime();
            }
            lastDataPacketReceptionTime = simTime();
        }
        // or unicast via this node
        else if (packet->getVia() == nodeId) {
            bubble("I received a unicast data packet to forward!");
            manageReceivedDataPacketToForward(msg);
            if (firstDataPacketReceptionTime == 0) {
                firstDataPacketReceptionTime = simTime();
            }
            lastDataPacketReceptionTime = simTime();
        }
        // or not, if it's a unicast packet we just happened to receive.
        else {
            bubble("Unicast message not for me!");
            receivedDataPackets++;
            receivedDataPacketsNotForMe++;
            EV_INFO << "Unicast data packet not for me: via=" << packet->getVia() << endl;
            lastDataPacketReceptionTime = simTime();
        }
    }
}

void LoRaNodeApp::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        handleSelfMessage(msg);
    } else {
        handleMessageFromLowerLayer(msg);
    }
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation,
        IDoneCallback *doneCallback) {
    Enter_Method_Silent();

    if (dynamic_cast<ModuleCrashOperation *>(operation)) {
        EV_INFO << "LoRa Node crashed!";
        cleanTeardown();
    }
    else
        throw cRuntimeError("Unsupported lifecycle operation '%s'",
                            operation->getClassName());
    return true;
}

void LoRaNodeApp::manageReceivedRoutingPacket(cMessage *msg) {

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    // Check it actually is a routing message
    if (packet->getMsgType() == ROUTING) {

        receivedRoutingPackets++;

        sanitizeRoutingTable();

        switch (routingMetric) {

            // The node performs no forwarding
            case NO_FORWARDING:
                bubble("Discarding routing packet as forwarding is disabled");
                break;

            // Forwarding is broadcast-based
            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
                bubble("Discarding routing packet as forwarding is broadcast-based");
                break;

            // Single-SF metrics
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:

                bubble("Processing routing packet");

                // Add new route to the neighbour node that sent this routing packet...
                if (!isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource()) ) {
                    EV << "Adding neighbour " << packet->getSource() << endl;
                    singleMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.valid = simTime() + routeTimeout;
                    switch (routingMetric) {
                        case HOP_COUNT_SINGLE_SF:
                            newNeighbour.metric = 1;
                            break;
                        case RSSI_SUM_SINGLE_SF:
                        case RSSI_PROD_SINGLE_SF:
                            newNeighbour.metric = std::abs(packet->getOptions().getRSSI());
                            break;
                        case ETX_SINGLE_SF:
                            newNeighbour.metric = 1;
                            newNeighbour.window[0] = packet->getDataInt();
                            // Fill window[1] and beyond, until windowSizeth element, with 0's
                            for (int i = 1; i<windowSize; i++) {
                                newNeighbour.window[i] = 0;
                            }
                            break;
                        }
                    singleMetricRoutingTable.push_back(newNeighbour);
                }

                // ... or refresh route to known neighbour.
                else {
                    int routeIndex = getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource());
                    if (routeIndex >= 0) {
                        singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        // Besides the route timeout, each metric may need different things to be updated
                        int metricValue = 1;
                        switch (routingMetric) {
                            case RSSI_SUM_SINGLE_SF:
                            case RSSI_PROD_SINGLE_SF:
                                singleMetricRoutingTable[routeIndex].metric = std::abs(packet->getOptions().getRSSI());
                                break;
                            case ETX_SINGLE_SF:
                                // Metric must be recalculated and ETX window must be updated
                                // Calculate the metric based on the window of previously received routing packets and update it
                                for (int i=0; i<windowSize; i++) {
                                    metricValue = metricValue + (packet->getDataInt() - (singleMetricRoutingTable[routeIndex].window[i] + i + 1));
                                }
                                singleMetricRoutingTable[routeIndex].metric = std::max(1, metricValue);
                                // Update the window (set nth element to nth-1 element, and 0th element to the current packet DataInt)
                                for (int i=windowSize-1; i>0; i--) {
                                    singleMetricRoutingTable[routeIndex].window[i] = singleMetricRoutingTable[routeIndex].window[i-1];
                                }
                                singleMetricRoutingTable[routeIndex].window[0] = packet->getDataInt();
                                break;
                            default:
                                break;
                        }
                     }
                }

                // Iterate the routes in the incoming packet. Add new ones to the routing table, or update known ones.
                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId) {
                        // Add new route...
                        if (!isRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource())) {
                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << endl;

                            singleMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.valid = simTime() + routeTimeout;
                            switch(routingMetric) {
                            case HOP_COUNT_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()+1;
                                break;
                            case RSSI_SUM_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                                break;
                            case RSSI_PROD_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                                break;
                            case ETX_SINGLE_SF:
                                newRoute.metric = \
                                    singleMetricRoutingTable[getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource())].metric \
                                    + thisRoute.getPriMetric();
                                break;
                            default:
                                break;
                            }
                            singleMetricRoutingTable.push_back(newRoute);
                        }

                        // ... or update a known one.
                        else {
                            int routeIndex = getRouteIndexInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource());
                            if (routeIndex >= 0) {
                                // Update route timeout
                                singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                                // Besides the route timeout, each metric may need different things to be updated
                                switch (routingMetric) {
                                    case HOP_COUNT_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+1;
                                        break;
                                    case RSSI_SUM_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                                        break;
                                    case RSSI_PROD_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                                        break;
                                    case ETX_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric() \
                                            + singleMetricRoutingTable[getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource())].metric;
                                        break;
                                    default:
                                        break;
                                }
                            }
                        }
                    }
                }
                break;

            // Multi-SF metrics
            case TIME_ON_AIR_NEWEST_CAD_MULTI_SF:
            case TIME_ON_AIR_RANDOM_CAD_MULTI_SF:
            case TIME_ON_AIR_HC_CAD_MULTI_SF:
            case TIME_ON_AIR_ETX_CAD_MULTI_SF:
            case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
            case TIME_ON_AIR_RMP1_CAD_MULTI_SF:

                bubble("Processing routing packet");

                // Add new route to the neighbour node that sent this routing packet...
                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.valid = simTime() + routeTimeout;
                    newNeighbour.priMetric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    newNeighbour.secMetric = 1;
                    switch (routingMetric) {
                        case TIME_ON_AIR_ETX_CAD_MULTI_SF:
                            newNeighbour.window[0] = packet->getDataInt();
                            // Fill window[1] and beyond, until windowSizeth element, with 0's
                            for (int i = 1; i<windowSize; i++) {
                                newNeighbour.window[i] = 0;
                            }
                            break;
                        case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
                            newNeighbour.priMetric = newNeighbour.priMetric * ( numberOfNodes/( std::max(numberOfNodes-packet->getBuffer(), numberOfNodes-1)) );
                            break;
                        default:
                            break;
                    }
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                // ... or refresh route to known neighbour.
                else {
                    int routeIndex = getRouteIndexInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF());
                    if (routeIndex >= 0) {
                        dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        // Besides the route timeout, each metric may need different things to be calculated
                        int metricValue = pow(2, packet->getOptions().getLoRaSF() - 7);
                        int etx = 1;
                        switch (routingMetric) {
                            case TIME_ON_AIR_ETX_CAD_MULTI_SF:
                                // Metric must be recalculated and ETX window must be updated
                                // Calculate the metric based on the window of previously received routing packets and update it
                                for (int i=0; i<windowSize; i++) {
                                    etx = etx + (packet->getDataInt() - (dualMetricRoutingTable[routeIndex].window[i] + i+1));
                                }
                                dualMetricRoutingTable[routeIndex].priMetric = std::max(metricValue, metricValue*etx);
                                // Update the window (set nth element to nth-1 element, and 0th element to the current packet DataInt)
                                for (int i=windowSize-1; i>0; i--) {
                                    dualMetricRoutingTable[routeIndex].window[i] = dualMetricRoutingTable[routeIndex].window[i-1];
                                }
                                dualMetricRoutingTable[routeIndex].window[0] = packet->getDataInt();
                                break;
                            case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
                                // Metric must be recalculated based on current buffer occupation
                                dualMetricRoutingTable[routeIndex].priMetric = pow(2, packet->getOptions().getLoRaSF() - 7) * ( numberOfNodes/( std::max(numberOfNodes-packet->getBuffer(), numberOfNodes-1)) );
                                break;
                            default:
                                break;
                        }
                    }
                }

                // Iterate the routes in the incoming packet. Add new ones to the routing table, or update known ones.
                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add a new route...
                        if ( !isRouteInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF())) {
                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.valid = simTime() + routeTimeout;
                            newRoute.priMetric = thisRoute.getPriMetric() \
                                + dualMetricRoutingTable[getRouteIndexInDualMetricRoutingTable(packet->getSource(),  packet->getSource(), packet->getOptions().getLoRaSF())].priMetric;
                            newRoute.secMetric = thisRoute.getSecMetric() \
                                + dualMetricRoutingTable[getRouteIndexInDualMetricRoutingTable(packet->getSource(),  packet->getSource(), packet->getOptions().getLoRaSF())].secMetric;
                            dualMetricRoutingTable.push_back(newRoute);
                        }
                    }

                    // ... or update a known one.
                    else {
                        int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                        if (routeIndex >= 0) {
                            dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() \
                                + dualMetricRoutingTable[getRouteIndexInDualMetricRoutingTable(packet->getSource(),  packet->getSource(), packet->getOptions().getLoRaSF())].priMetric;
                            dualMetricRoutingTable[routeIndex].secMetric = thisRoute.getSecMetric() \
                                + dualMetricRoutingTable[getRouteIndexInDualMetricRoutingTable(packet->getSource(),  packet->getSource(), packet->getOptions().getLoRaSF())].secMetric;
                        }
                    }
                } // End of multiSF routes for loop.

                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            default:
                break;
        } // End of routingMetric switch
        routingTableSize.collect(singleMetricRoutingTable.size());
    } // End of routing packet type if

    EV << "## Routing table at node " << nodeId << "##" << endl;
    for (int i=0; i<singleMetricRoutingTable.size(); i++) {
        EV << "Node " << singleMetricRoutingTable[i].id << " via " << singleMetricRoutingTable[i].via << " with cost " << singleMetricRoutingTable[i].metric << endl;
    }
}

void LoRaNodeApp::manageReceivedPacketToForward(cMessage *msg) {
    receivedPacketsToForward++;

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        manageReceivedDataPacketToForward(msg);
        break;
    // ACK packet
    case ACK:
        manageReceivedAckPacketToForward(msg);
        break;
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedAckPacketToForward(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsToForward++;
}

void LoRaNodeApp::manageReceivedDataPacketToForward(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsToForward++;
    bool newPacketToForward = false;

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    auto dataPacket = staticPtrCast<LoRaAppPacket>(packet->dupShared());

    // Check for too old packets with TTL <= 1
    if (packet->getTtl() <= 1) {
        bubble("This packet has reached TTL expiration!");
        receivedDataPacketsToForwardExpired++;
    }

    // Packet has not reached its maximum TTL
    else {
        receivedDataPacketsToForwardCorrect++;

        switch (routingMetric) {
            case NO_FORWARDING:
                bubble("Discarding packet as forwarding is disabled");
                break;

            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_RMP1_CAD_MULTI_SF:
            case TIME_ON_AIR_HC_CAD_MULTI_SF:
            case TIME_ON_AIR_ETX_CAD_MULTI_SF:
            default:
                // Check if the packet has already been forwarded
                if (isPacketForwarded(*packet)) {
                    bubble("This packet has already been forwarded!");
                    forwardPacketsDuplicateAvoid++;
                }
                // Check if the packet is buffered to be forwarded
                else if (isPacketToBeForwarded(*packet)) {
                    bubble("This packet is already scheduled to be forwarded!");
                    forwardPacketsDuplicateAvoid++;
                // A previously-unknown packet has arrived
                } else {
                    bubble("Saving packet to forward it later!");
                    receivedDataPacketsToForwardUnique++;

                    dataPacket->setTtl(packet->getTtl() - 1);
                    if (packetsToForwardMaxVectorSize == 0 || LoRaPacketsToForward.size()<packetsToForwardMaxVectorSize) {
                        LoRaPacketsToForward.push_back(*dataPacket);
                        newPacketToForward = true;
                    }
                    else {
                        forwardBufferFull++;
                    }
                }
        }

    }

    if (newPacketToForward) {
        // Reschedule the selfPacketTxMsg as soon as possible (honoring dutyCycle via minNextPacketTransmissionTime)
        simtime_t nextScheduleTime = math::maxnan(simTime().dbl(), minNextPacketTransmissionTime.dbl()) + 10*simTimeResolution;
        rescheduleAt(nextScheduleTime, selfPacketTxMsg);
    }
}

void LoRaNodeApp::manageReceivedPacketForMe(cMessage *msg) {
    receivedPacketsForMe++;

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        manageReceivedDataPacketForMe(msg);
        break;
        // ACK packet
    case ACK:
        manageReceivedAckPacketForMe(msg);
        break;
        // Other type
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedDataPacketForMe(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsForMe++;

    auto pkt = check_and_cast<Packet *>(msg);
    auto packet = pkt->removeAtFront<LoRaAppPacket>();
    dataPacketsForMeLatency.collect(simTime()-packet->getDepartureTime());

    if (isDataPacketForMeUnique(*packet)) {
        DataPacketsForMe.push_back(*packet);
        receivedDataPacketsForMeUnique++;
        dataPacketsForMeUniqueLatency.collect(simTime()-packet->getDepartureTime());
    }
}

void LoRaNodeApp::manageReceivedAckPacketForMe(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsForMe++;

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();

    EV_INFO << "Fire alarm received from node: "
            << packet->getSource()
            << " temperature: "
            << packet->getDataInt()
            << endl;
}

std::pair<Packet *, simtime_t> LoRaNodeApp::getDataPacketToSend() {
    bool transmit = false;
    simtime_t txDuration = 0;
    std::string fullName = "";
    auto dataPacket = makeShared<LoRaAppPacket>();
    Packet *dataPkt = nullptr;
    int customSF = -1; // If -1 use getSF()

    // Send cached data packet from previous attempt
    if (cachedDataPktToSendInfo.first != nullptr) {
        auto pktInfo = cachedDataPktToSendInfo;
        cachedDataPktToSendInfo.first = nullptr;
        EV_INFO << "Getting cached data packet" << endl;
        return pktInfo;
    }
    // Send local data packets with a configurable ownDataPriority priority over packets to forward, if there is any
    else if ((LoRaPacketsToSend.size() > 0 && bernoulli(ownDataPriority))
             || (LoRaPacketsToSend.size() > 0 && LoRaPacketsToForward.size() == 0)) {

        // Name packets to ease tracking
        const char* addName = "Orig";
        fullName += addName;
        fullName += std::to_string(nodeId);

        // Get the data from the first packet in the data buffer to send it
        dataPacket->setMsgType(LoRaPacketsToSend.front().getMsgType());
        dataPacket->setDataInt(LoRaPacketsToSend.front().getDataInt());
        dataPacket->setSource(LoRaPacketsToSend.front().getSource());
        dataPacket->setVia(LoRaPacketsToSend.front().getSource());
        dataPacket->setDestination(LoRaPacketsToSend.front().getDestination());
        dataPacket->setTtl(LoRaPacketsToSend.front().getTtl());
        LoRaOptions opts = dataPacket->getOptions();
        opts.setAppACKReq(LoRaPacketsToSend.front().getOptions().getAppACKReq());
        dataPacket->setOptions(opts);
        dataPacket->setChunkLength(B(LoRaPacketsToSend.front().getChunkLength()));
        dataPacket->setDepartureTime(simTime());

        addName = "Dest";
        fullName += addName;
        fullName += std::to_string(dataPacket->getDestination());

        LoRaPacketsToSend.erase(LoRaPacketsToSend.begin());

        transmit = true;

        sentDataPackets++;
        if (firstDataPacketTransmissionTime == 0)
            firstDataPacketTransmissionTime = simTime();
        lastDataPacketTransmissionTime = simTime();
    }
    // Forward other nodes' packets, if any
    else if (LoRaPacketsToForward.size() > 0) {

        const char* addName = "Fwd";
        fullName += addName;
        fullName += std::to_string(nodeId);

        switch (routingMetric) {
            case NO_FORWARDING:
                // This should never happen
                bubble("Forwarding disabled!");
                break;

            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_NEWEST_CAD_MULTI_SF:
            case TIME_ON_AIR_RANDOM_CAD_MULTI_SF:
            case TIME_ON_AIR_HC_CAD_MULTI_SF:
            case TIME_ON_AIR_ETX_CAD_MULTI_SF:
            case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
            case TIME_ON_AIR_RMP1_CAD_MULTI_SF:
            default:
                // TODO: Investigate while loop but single transmit
                while (LoRaPacketsToForward.size() > 0) {
                    addName = "FWD-";
                    fullName += addName;
                    fullName += std::to_string(routingMetric);
                    addName = "-";
                    fullName += addName;

                    // Get the data from the first packet in the forwarding buffer to send it
                    dataPacket->setMsgType(LoRaPacketsToForward.front().getMsgType());
                    dataPacket->setDataInt(LoRaPacketsToForward.front().getDataInt());
                    dataPacket->setSource(LoRaPacketsToForward.front().getSource());
                    dataPacket->setVia(LoRaPacketsToForward.front().getSource());
                    dataPacket->setDestination(LoRaPacketsToForward.front().getDestination());
                    dataPacket->setTtl(LoRaPacketsToForward.front().getTtl());
                    LoRaOptions opts = dataPacket->getOptions();
                    opts.setAppACKReq(LoRaPacketsToForward.front().getOptions().getAppACKReq());
                    dataPacket->setOptions(opts);
                    dataPacket->setChunkLength(B(LoRaPacketsToForward.front().getChunkLength()));
                    dataPacket->setDepartureTime(LoRaPacketsToForward.front().getDepartureTime());

                    // Erase the first packet in the forwarding buffer
                    LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                    // Redundantly check that the packet has not been forwarded in the mean time, which should never occur
                    if (!isPacketForwarded(*dataPacket)) {
                        bubble("Forwarding packet!");
                        forwardedPackets++;
                        forwardedDataPackets++;
                        transmit = true;

                        // Keep a copy of the forwarded packet to avoid sending it again if received later on
                        LoRaPacketsForwarded.push_back(*dataPacket);
                        if (LoRaPacketsForwarded.size() > forwardedPacketVectorSize){
                            LoRaPacketsForwarded.erase(LoRaPacketsForwarded.begin());
                        }
                        break;
                    }
                }
                break;
        }
    }

    if (transmit) {
        const char* ownName = "Tx";
        fullName += ownName;

        sanitizeRoutingTable();

        int routeIndex = getBestRouteIndexTo(dataPacket->getDestination());

        switch (routingMetric) {
            case FLOODING_BROADCAST_SINGLE_SF:
                dataPacket->setVia(BROADCAST_ADDRESS);

            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
                if ( routeIndex >= 0 )
                    // dataPacket->setVia(BROADCAST_ADDRESS);
                    dataPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                else {
                    dataPacket->setVia(BROADCAST_ADDRESS);
                }
                break;
            case TIME_ON_AIR_RMP1_CAD_MULTI_SF:
                // Randomly pick a higher SF than needed for this route
                if (routeIndex >= 0)
                    customSF = pickCADSF(dualMetricRoutingTable[routeIndex].sf);
                else
                    customSF = pickCADSF(minLoRaSF);
                // do not break;
            case TIME_ON_AIR_NEWEST_CAD_MULTI_SF:
            case TIME_ON_AIR_RANDOM_CAD_MULTI_SF:
            case TIME_ON_AIR_HC_CAD_MULTI_SF:
            case TIME_ON_AIR_ETX_CAD_MULTI_SF:
            case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
            default:
                if ( routeIndex >= 0 ) {
                    dataPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                    customSF = dualMetricRoutingTable[routeIndex].sf;
                }
                else {
                    dataPacket->setVia(BROADCAST_ADDRESS);
                }
                break;
        }

        dataPkt = new Packet(fullName.c_str());
        dataPkt->insertAtFront(dataPacket);
        setLoRaTagToPkt(dataPkt, customSF);
        txDuration = calculateTransmissionDuration(dataPkt);
    }

    return std::make_pair(transmit ? dataPkt : nullptr, txDuration);
}

std::pair<Packet *, simtime_t> LoRaNodeApp::getRoutingPacketToSend() {
    bool transmit = false;
    simtime_t txDuration = 0;
    int numberOfRoutes = 0;
    auto routingPacket = makeShared<LoRaAppPacket>();
    Packet *routingPkt = nullptr;

    // Send cached routing packet from previous attempt
    if (cachedRoutingPktToSendInfo.first != nullptr) {
        auto pktInfo = cachedRoutingPktToSendInfo;
        cachedRoutingPktToSendInfo.first = nullptr;
        EV_INFO << "Getting cached routing packet" << endl;
        return pktInfo;
    }

    sanitizeRoutingTable();

    std::vector<LoRaRoute> theseLoRaRoutes;
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    switch (routingMetric) {

        case NO_FORWARDING:
            break;

        case FLOODING_BROADCAST_SINGLE_SF:
        case SMART_BROADCAST_SINGLE_SF:
            break;

        // Single-SF metrics
        case HOP_COUNT_SINGLE_SF:
        case RSSI_SUM_SINGLE_SF:
        case RSSI_PROD_SINGLE_SF:
        case ETX_SINGLE_SF:

            transmit = true;

            // Count the number of best routes
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0) {
                        numberOfRoutes++;
                    }
                }
            }

            // Make room for numberOfRoutes routes
            routingPacket->setRoutingTableArraySize(numberOfRoutes);

            // Add the best route to each node to the routing packet
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0) {

                        LoRaRoute thisLoRaRoute;
                        thisLoRaRoute.setId(singleMetricRoutingTable[getBestRouteIndexTo(i)].id);
                        thisLoRaRoute.setPriMetric(singleMetricRoutingTable[getBestRouteIndexTo(i)].metric);
                        routingPacket->setRoutingTable(numberOfRoutes-1, thisLoRaRoute);
                        numberOfRoutes--;
                    }
                }
            }

            break;

        // Multi-SF metrics
        case TIME_ON_AIR_NEWEST_CAD_MULTI_SF:
        case TIME_ON_AIR_RANDOM_CAD_MULTI_SF:
        case TIME_ON_AIR_HC_CAD_MULTI_SF:
        case TIME_ON_AIR_ETX_CAD_MULTI_SF:
        case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
        case TIME_ON_AIR_RMP1_CAD_MULTI_SF:
            transmit = true;

            // Count the number of best routes
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0) {
                        numberOfRoutes++;
                    }
                }
            }

            // Save the number of routes to the packet
            routingPacket->setRoutingTableArraySize(numberOfRoutes);

            std::vector<LoRaRoute> allLoRaRoutes;

            //routingPacket->setRoutingTableArraySize(dualMetricRoutesCount);

            // Add the best route to each node to the routing packet
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0 ){
                        LoRaRoute thisLoRaRoute;
                        thisLoRaRoute.setId(dualMetricRoutingTable[getBestRouteIndexTo(i)].id); //i.e., "i"
                        thisLoRaRoute.setPriMetric(dualMetricRoutingTable[getBestRouteIndexTo(i)].priMetric);
                        thisLoRaRoute.setSecMetric(dualMetricRoutingTable[getBestRouteIndexTo(i)].secMetric);
                        routingPacket->setRoutingTable(numberOfRoutes-1, thisLoRaRoute);
                        numberOfRoutes--;
                    }
                }
            }

           // Decide on what SF to transmit and the packet is done
           setSF(pickCADSF(minLoRaSF));

           break;
    }

    if (transmit) {
        //add LoRa control info
        routingPacket->setMsgType(ROUTING);
        routingPacket->setDataInt(sentRoutingPackets);
        routingPacket->setSource(nodeId);
        routingPacket->setVia(nodeId);
        routingPacket->setDestination(BROADCAST_ADDRESS);
        LoRaOptions opts = routingPacket->getOptions();
        opts.setAppACKReq(false);
        routingPacket->setOptions(opts);
        routingPacket->setChunkLength(B(routingPacketMaxSize));
        routingPacket->setDepartureTime(simTime());

        routingPkt = new Packet("RoutingPacket");
        routingPkt->insertAtFront(routingPacket);
        setLoRaTagToPkt(routingPkt);
        txDuration = calculateTransmissionDuration(routingPkt);
    }
    return std::make_pair(transmit ? routingPkt : nullptr, txDuration);
}

void LoRaNodeApp::sendDataPacket(Packet *pkt) {
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    auto tag = pkt->getTag<LoRaTag>();
    int sf = tag->getSpreadFactor();
    int tp = (int)math::mW2dBmW(tag->getPower().get());
    bool localData = (packet->getSource() == nodeId);

    if (packet->getVia() == BROADCAST_ADDRESS) {
        if (localData)
            broadcastDataPackets++;
        else
            broadcastForwardedPackets++;
    }

    if (localData) {
        EV_INFO << "Sending data packet" << endl;
        // bubble("Sending data packet");
    }
    else {
        EV_INFO << "Forwarding data packet" << endl;
        // bubble("Forwarding data packet");
    }

    send(pkt, "socketOut");
    sentPackets++;

    allTxPacketsSFStats.collect(sf);
    if (localData) {
        owndataTxPacketsSFStats.collect(sf);
    }
    else {
        fwdTxPacketsSFStats.collect(sf);
    }
    txSfVector.record(sf);
    txTpVector.record(tp);
    emit(LoRa_AppPacketSent, sf);
}

void LoRaNodeApp::sendRoutingPacket(Packet *pkt) {
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    auto tag = pkt->getTag<LoRaTag>();
    int sf = tag->getSpreadFactor();
    int tp = (int)math::mW2dBmW(tag->getPower().get());

    EV_INFO << "Sending routing packet" << endl;
    // bubble("Sending routing packet");
    send(pkt, "socketOut");
    sentPackets++;
    sentRoutingPackets++;

    allTxPacketsSFStats.collect(sf);
    routingTxPacketsSFStats.collect(sf);
    txSfVector.record(sf);
    txTpVector.record(tp);
    emit(LoRa_AppPacketSent, sf);
}

void LoRaNodeApp::increaseSFIfPossible() {
    if (getSF() < 12) {
        // char text[32];
        // sprintf(text, "Increasing SF from %d to %d", loRaSF, loRaSF+1);
        // bubble(text);
        setSF(getSF()+1);
    }
}

bool LoRaNodeApp::isNeighbour(int neighbourId) {
    for (std::vector<int>::iterator nbptr = neighbourNodes.begin();
            nbptr < neighbourNodes.end(); nbptr++) {
        if (neighbourId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isRouteInSingleMetricRoutingTable(int id, int via) {
    if (getRouteIndexInSingleMetricRoutingTable(id, via) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInSingleMetricRoutingTable(int id, int via) {
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

    for (int i = 0; i < singleMetricRoutesCount; i++) {
        if (singleMetricRoutingTable[i].id == id && singleMetricRoutingTable[i].via == via) {
            return i;
        }
    }

    return -1;
}

bool LoRaNodeApp::isRouteInDualMetricRoutingTable(int id, int via, int sf) {
    if (getRouteIndexInDualMetricRoutingTable(id, via, sf) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInDualMetricRoutingTable(int id, int via, int sf) {
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    for (int i = 0; i < dualMetricRoutesCount; i++) {
        if (dualMetricRoutingTable[i].id == id && dualMetricRoutingTable[i].via == via && dualMetricRoutingTable[i].sf == sf) {
            return i;
        }
    }

    return -1;
}


bool LoRaNodeApp::isKnownNode(int knownNodeId) {
    for (std::vector<int>::iterator nbptr = knownNodes.begin();
            nbptr < knownNodes.end(); nbptr++) {
        if (knownNodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isACKed(int nodeId) {
    for (std::vector<int>::iterator nbptr = ACKedNodes.begin();
            nbptr < ACKedNodes.end(); nbptr++) {
        if (nodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketForwarded(const LoRaAppPacket &packet) {

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        if (packet.getMsgType() == lbptr->getMsgType()
                && packet.getDataInt() == lbptr->getDataInt()
                && packet.getSource() == lbptr->getSource()
                && packet.getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketToBeForwarded(const LoRaAppPacket &packet) {

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        if (packet.getMsgType() == lbptr->getMsgType()
                && packet.getDataInt() == lbptr->getDataInt()
                && packet.getSource() == lbptr->getSource()
                && packet.getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isDataPacketForMeUnique(const LoRaAppPacket &packet) {

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        if (packet.getMsgType() == lbptr->getMsgType()
                && packet.getDataInt() == lbptr->getDataInt()
                && packet.getSource() == lbptr->getSource()
                && packet.getDestination() == lbptr->getDestination()) {
            return false;
        }
    }
    return true;
}

int LoRaNodeApp::pickCADSF(int lowestSF) {
    if (lowestSF < minLoRaSF)
        lowestSF = minLoRaSF;
    do {
        int thisSF = intuniform(lowestSF,maxLoRaSF);
        if (bernoulli(pow(0.5, thisSF-lowestSF+1)))
            return thisSF;
    } while (true);
}

int LoRaNodeApp::getBestRouteIndexTo(int destination) {
    if (singleMetricRoutingTable.size() > 0) {

        int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

        std::vector<singleMetricRoute> availableRoutes;

        for (int i = 0; i < singleMetricRoutesCount; i++) {
            if (singleMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(singleMetricRoutingTable[i]);

            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;
            int bestMetric = availableRoutes[0].metric;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);
            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].metric < bestMetric) {
                    bestMetric = availableRoutes[j].metric;
                }
            }

            simtime_t lastMetric = 0;

            for (int k = 0; k < availableRoutesCount; k++) {
                if (availableRoutes[k].metric == bestMetric) {
                    if (availableRoutes[k].valid >= lastMetric) {
                        bestRoute = k;
                        lastMetric = availableRoutes[k].valid;
                    }
                }
            }
            return getRouteIndexInSingleMetricRoutingTable(availableRoutes[bestRoute].id, availableRoutes[bestRoute].via);
        }
    } // End single-SF metrics

    else if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = dualMetricRoutingTable.size();

        std::vector<dualMetricRoute> availableRoutes;
        std::vector<dualMetricRoute> tieRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);
            int tieRoutesCount = 0;
            int j;

            switch (routingMetric) {
                case TIME_ON_AIR_NEWEST_CAD_MULTI_SF:
                    // Find best priMetric with longest validity, no ties expected
                    for (j = 0; j < availableRoutesCount; j++) {
                        if ( availableRoutes[j].priMetric < availableRoutes[bestRoute].priMetric ||
                                ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                        availableRoutes[j].valid > availableRoutes[bestRoute].valid) ) {
                            bestRoute = j;
                        }
                    }
                    break;
                case TIME_ON_AIR_RANDOM_CAD_MULTI_SF:
                    // Find best routes by priMetric, choose randomly from ties.
                    // Find best priMetric value
                    for (j = 0; j < availableRoutesCount; j++) {
                        if (availableRoutes[j].priMetric < availableRoutes[bestRoute].priMetric) {
                            bestRoute = j;
                        }
                    }
                    // Find ties
                    for (j = 0; j < availableRoutesCount; j++) {
                        if (availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric) {
                            tieRoutes.push_back(availableRoutes[j]);
                        }
                    }
                    // Count ties
                    tieRoutesCount = end(tieRoutes) - begin(tieRoutes);
                    // Return a random route among the ties
                    if (tieRoutesCount > 0) {
                        int bestTieRoute = intuniform(0, tieRoutesCount-1);
                        return getRouteIndexInDualMetricRoutingTable(tieRoutes[bestTieRoute].id, tieRoutes[bestTieRoute].via, tieRoutes[bestTieRoute].sf);
                    }
                    break;
                case TIME_ON_AIR_HC_CAD_MULTI_SF:
                case TIME_ON_AIR_ETX_CAD_MULTI_SF:
                case TIME_ON_AIR_FQUEUE_CAD_MULTI_SF:
                case TIME_ON_AIR_RMP1_CAD_MULTI_SF:
                default:
                    for (j = 0; j < availableRoutesCount; j++) {
                        if (availableRoutes[j].priMetric < availableRoutes[bestRoute].priMetric ||
                                ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                        availableRoutes[j].secMetric < availableRoutes[bestRoute].secMetric) ||
                                        ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                                availableRoutes[j].secMetric == availableRoutes[bestRoute].secMetric &&
                                                availableRoutes[j].valid > availableRoutes[bestRoute].valid)) {
                            bestRoute = j;
                        }
                    }
                    break;
            }

            return getRouteIndexInDualMetricRoutingTable(availableRoutes[bestRoute].id, availableRoutes[bestRoute].via, availableRoutes[bestRoute].sf);
        }
    } // End mulit-SF metrics

    // If no route found, or no routes at all
    return -1;
}

void LoRaNodeApp::sanitizeRoutingTable() {
    bool routeDeleted = false;

    if (singleMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<singleMetricRoute>::iterator smr =
                    singleMetricRoutingTable.begin(); smr < singleMetricRoutingTable.end();
                    smr++) {
                if (smr->valid < simTime()) {
                    singleMetricRoutingTable.erase(smr);
                    routeDeleted = true;
                    deletedRoutes++;
                    break;
                }
            }
        } while (routeDeleted);
    }
    else if (dualMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<dualMetricRoute>::iterator dmr =
                    dualMetricRoutingTable.begin(); dmr < dualMetricRoutingTable.end();
                    dmr++) {
                if (dmr->valid < simTime()) {
                    dualMetricRoutingTable.erase(dmr);
                    routeDeleted = true;
                    deletedRoutes++;
                    break;
                }
            }
        } while (routeDeleted);
    }
}

int LoRaNodeApp::getSFTo(int destination) {
    if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        std::vector<dualMetricRoute> availableRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);

            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].sf < availableRoutes[bestRoute].sf) {
                    bestRoute = j;
                }
            }

            if ( availableRoutes[bestRoute].sf >= minLoRaSF && availableRoutes[bestRoute].sf <= maxLoRaSF) {
                return availableRoutes[bestRoute].sf;
            }
        }
    }

    return minLoRaSF;
}


simtime_t LoRaNodeApp::calculateTransmissionDuration(cMessage *msg) {

    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    auto tag = pkt->getTag<LoRaTag>();

    int nPreamble = 8;
    simtime_t Tsym = (pow(2, tag->getSpreadFactor()))/(tag->getBandwidth().get()/1000);
    simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    int payloadBytes = B(packet->getChunkLength()).get()+8; //+8 bytes for headers

    int payloadSymbNb = 8;
    payloadSymbNb += std::ceil((8*payloadBytes - 4*tag->getSpreadFactor() + 28 + 16 - 20*0)/(4*(tag->getSpreadFactor()-2*0)))*(tag->getCodeRendundance() + 4);
    if (payloadSymbNb < 8) payloadSymbNb = 8;

    simtime_t Theader = 0.5 * (8+payloadSymbNb) * Tsym / 1000;
    simtime_t Tpayload = 0.5 * (8+payloadSymbNb) * Tsym / 1000;

    const simtime_t duration = Tpreamble + Theader + Tpayload;
    return duration;
}

simtime_t LoRaNodeApp::getTimeToNextRoutingPacket() {
    if ( strcmp(par("timeToNextRoutingPacketDist").stringValue(), "uniform") == 0) {
        simtime_t routingTime = uniform(timeToNextRoutingPacketMin, timeToNextRoutingPacketMax);
        return routingTime;
    }
    else if ( strcmp(par("timeToNextRoutingPacketDist").stringValue(), "exponential") == 0) {
        simtime_t routingTime = exponential(timeToNextRoutingPacketAvg);
        return routingTime;
    }
    return simTime();
}

void LoRaNodeApp::setSF(int SF) {
    loRaRadio->loRaSF = SF;
}

int LoRaNodeApp::getSF() {
    return loRaRadio->loRaSF;
}

void LoRaNodeApp::setTP(int TP) {
    loRaRadio->loRaTP = TP;
}

double LoRaNodeApp::getTP() {
    return loRaRadio->loRaTP;
}

void LoRaNodeApp::setCF(units::values::Hz CF) {
    loRaRadio->loRaCF = CF;
}

units::values::Hz LoRaNodeApp::getCF() {
    return loRaRadio->loRaCF;
}

void LoRaNodeApp::setBW(units::values::Hz BW) {
    loRaRadio->loRaBW = BW;
}

units::values::Hz LoRaNodeApp::getBW() {
    return loRaRadio->loRaBW;
}

void LoRaNodeApp::setCR(int CR) {
    loRaRadio->loRaCR = CR;
}

int LoRaNodeApp::getCR() {
    return loRaRadio->loRaCR;
}

const char *LoRaNodeApp::getAppModeName(AppMode appMode)
{
    if (!appModeEnum)
        appModeEnum = cEnum::get(opp_typename(typeid(LoRaNodeApp::AppMode)));
    return appModeEnum->getStringFor(appMode) + 11;
}

void LoRaNodeApp::setLoRaTagToPkt(Packet *packet, int customSF) {
    auto loraTag = packet->addTagIfAbsent<LoRaTag>();
    loraTag->setBandwidth(getBW());
    loraTag->setCenterFrequency(getCF());
    loraTag->setSpreadFactor((customSF != -1) ? customSF : getSF());
    loraTag->setCodeRendundance(getCR());
    loraTag->setPower(mW(math::dBmW2mW(getTP())));
}

void LoRaNodeApp::setCommActiveState(bool state) {
    if (state != commActiveState) {
        emit(commActiveChangedSignal, (intval_t)state);
        commActiveState = state;
    }
}

} //end namespace inet

