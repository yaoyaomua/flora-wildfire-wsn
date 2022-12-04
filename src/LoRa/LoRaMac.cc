//
// Copyright (C) 2016 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/common/ModuleAccess.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/UserPriority.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"
#include "LoRaMac.h"
#include "LoRaTagInfo_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"


namespace flora {

Define_Module(LoRaMac);

LoRaMac::~LoRaMac()
{
    cancelAndDelete(endTransmission);
    cancelAndDelete(endReception);
    cancelAndDelete(droppedPacket);
    cancelAndDelete(startActive);
    cancelAndDelete(endActive);
    cancelAndDelete(mediumStateChange);
}

/****************************************************************
 * Initialization functions.
 */
void LoRaMac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage 0\n";

        //maxQueueSize = par("maxQueueSize");
        headerLength = par("headerLength");
        ackLength = par("ackLength");
        slotTime = par("slotTime");
        cwMin = par("cwMin");
        cwMax = par("cwMax");
        ackTimeout = par("ackTimeout");
        retryLimit = par("retryLimit");

        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = MacAddress::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);

        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(LoRaRadio::droppedPacket, this);
        radio = check_and_cast<IRadio *>(radioModule);

        cModule *appModule = getParentModule()->getParentModule()->getSubmodule("app", 0);
        appModule->subscribe(LoRaNodeApp::commActiveChangedSignal, this);

        // initialize self messages
        endTransmission = new cMessage("EndTransmission");
        endReception = new cMessage("Reception");
        endBackoff = new cMessage("Backoff");
        droppedPacket = new cMessage("Dropped Packet");
        startActive = new cMessage("StartActive");
        endActive = new cMessage("EndActive");
        mediumStateChange = new cMessage("MediumStateChange");

        // set up internal queue
        txQueue = getQueue(gate(upperLayerInGateId));//check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));

        // state variables
        fsm.setName("LoRaMac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;

        // sequence number for messages
        sequenceNumber = 0;

        // statistics
        numRetry = 0;
        numSentWithoutRetry = 0;
        numGivenUp = 0;
        numCollision = 0;
        numSent = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;
        endActiveWhileReceiving = 0;
        endActiveWhileTransmitting = 0;

        // initialize watches
        if (getEnvir()->isGUI()) {
            WATCH(fsm);
            WATCH(backoffPeriod);
            WATCH(retryCounter);
            WATCH(numRetry);
            WATCH(numSentWithoutRetry);
            WATCH(numGivenUp);
            WATCH(numCollision);
            WATCH(numSent);
            WATCH(numReceived);
            WATCH(numSentBroadcast);
            WATCH(numReceivedBroadcast);
        }
    }
    else if (stage == INITSTAGE_LINK_LAYER)
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

void LoRaMac::finish()
{
    recordScalar("numRetry", numRetry);
    recordScalar("numSentWithoutRetry", numSentWithoutRetry);
    recordScalar("numGivenUp", numGivenUp);
    //recordScalar("numCollision", numCollision);
    recordScalar("numSent", numSent);
    recordScalar("numReceived", numReceived);
    recordScalar("numSentBroadcast", numSentBroadcast);
    recordScalar("numReceivedBroadcast", numReceivedBroadcast);
    recordScalar("endActiveWhileReceiving", endActiveWhileReceiving);
    recordScalar("endActiveWhileTransmitting", endActiveWhileTransmitting);
}

void LoRaMac::configureNetworkInterface()
{
    //NetworkInterface *e = new NetworkInterface(this);

    // data rate
    networkInterface->setDatarate(bitrate);
    networkInterface->setMacAddress(address);

    // capabilities
    //interfaceEntry->setMtu(par("mtu"));
    networkInterface->setMtu(std::numeric_limits<int>::quiet_NaN());
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
    networkInterface->setPointToPoint(false);
}

/****************************************************************
 * Message handling functions.
 */
void LoRaMac::handleSelfMessage(cMessage *msg)
{
    EV << "received self message: " << msg << endl;
    handleWithFsm(msg);
}
#if 0
void LoRaMac::handleUpperPacket(cMessage *msg)
{
    if(fsm.getState() != IDLE) {
            error("Wrong, it should not happen erroneous state: %s", fsm.getStateName());
    }
    auto pkt = check_and_cast<Packet *>(msg);

    pkt->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::lora);
//    LoRaMacControlInfo *cInfo = check_and_cast<LoRaMacControlInfo *>(msg->getControlInfo());
    auto pktEncap = encapsulate(pkt);

    const auto &frame = pktEncap->peekAtFront<LoRaMacFrame>();

    EV << "frame " << pktEncap << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;

    txQueue->enqueuePacket(pktEncap);
    if (fsm.getState() != IDLE)
        EV << "deferring upper message transmission in " << fsm.getStateName() << " state\n";
    else {
        popTxQueue();
        handleWithFsm(currentTxFrame);
    }
}
#endif
void LoRaMac::handleUpperPacket(Packet *packet)
{
    if(fsm.getState() != ACTIVE) {
         error("Wrong, it should not happen erroneous state: %s", fsm.getStateName());
    }
    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::apskPhy);

    EV << "frame " << packet << " received from higher layer " << endl;
    auto pktEncap = encapsulate(packet);
    const auto &frame = pktEncap->peekAtFront<LoRaMacFrame>();
    if (frame == nullptr)
        throw cRuntimeError("Header LoRaMacFrame not found");

    if (currentTxFrame != nullptr)
        throw cRuntimeError("Model error: incomplete transmission exists");
    currentTxFrame = pktEncap;
    handleWithFsm(currentTxFrame);
}


void LoRaMac::handleLowerPacket(Packet *msg)
{
    EV << "lower packet received in state " << fsm.getState() << endl;
    if (fsm.getState() == RECEIVING) {
        EV_INFO << "lower packet received, handling with handleWithFsm()" << endl;
        handleWithFsm(msg);
    }
    else {
        delete msg;
    }
}

void LoRaMac::processUpperPacket()
{
    Packet *packet = dequeuePacket();
    handleUpperMessage(packet);
}

queueing::IPassivePacketSource *LoRaMac::getProvider(cGate *gate)
{
    return (gate->getId() == upperLayerInGateId) ? txQueue.get() : nullptr;
}

void LoRaMac::handleCanPullPacketChanged(cGate *gate)
{
    Enter_Method("handleCanPullPacketChanged");
    if (fsm.getState() == ACTIVE && !txQueue->isEmpty()) {
        processUpperPacket();
    }
}

void LoRaMac::handlePullPacketProcessed(Packet *packet, cGate *gate, bool successful)
{
    Enter_Method("handlePullPacketProcessed");
    throw cRuntimeError("Not supported callback");
}

void LoRaMac::handleWithFsm(cMessage *msg)
{
    Ptr<LoRaMacFrame>frame = nullptr;

    auto pkt = dynamic_cast<Packet *>(msg);
    if (pkt) {
        const auto &chunk = pkt->peekAtFront<Chunk>();
        frame = dynamicPtrCast<LoRaMacFrame>(constPtrCast<Chunk>(chunk));
    }
    EV_INFO << "handling packet with handleWithFsm(): " << fsm << endl;

    if (msg == endActive &&
        fsm.getState() != ACTIVE &&
        fsm.getState() != RECEIVING &&
        fsm.getState() != TRANSMIT) {
        throw cRuntimeError("End Active triggered in unsupported state");
    }

    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            EV_INFO << "handling packet with handleWithFsm(): IDLE" << endl;
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Idle-Active,
                                  msg == startActive,
                                  ACTIVE,
            );
        }
        FSMA_State(ACTIVE)
        {
            EV_INFO << "handling packet with handleWithFsm(): ACTIVE" << endl;
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Active-Idle,
                                  msg == endActive,
                                  IDLE,
            );
            FSMA_Event_Transition(Defer-Transmit,
                                  isUpperMessage(msg) && !isMediumFree(),
                                  DEFER,
            );
            FSMA_Event_Transition(Start-Backoff,
                                  isUpperMessage(msg) && isMediumFree(),
                                  BACKOFF,
            );
            FSMA_Event_Transition(Start-Receiving,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING,
            );
        }
        FSMA_State(DEFER)
        {
            FSMA_Event_Transition(Start-Backoff,
                                  msg == mediumStateChange && isMediumFree(),
                                  BACKOFF,
            );
            FSMA_Event_Transition(Start-Receiving,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING,
            );
        }
        FSMA_State(BACKOFF)
        {
            FSMA_Enter(scheduleBackoffTimer());
            FSMA_Event_Transition(Start-Transmit,
                                  msg == endBackoff,
                                  TRANSMIT,
                invalidateBackoffPeriod();
            );
            FSMA_Event_Transition(Start-Receiving,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING,
                cancelBackoffTimer();
                decreaseBackoffPeriod();
            );
            FSMA_Event_Transition(Defer-Backoff,
                                  msg == mediumStateChange && !isMediumFree(),
                                  DEFER,
                cancelBackoffTimer();
                decreaseBackoffPeriod();
            );
        }
        FSMA_State(TRANSMIT)
        {
            EV_INFO << "handling packet with handleWithFsm(): TRANSMIT" << endl;
            FSMA_Enter(sendDataFrame(getCurrentTransmission()));
            FSMA_Event_Transition(Transmit-Active,
                                  msg == endTransmission,
                                  ACTIVE,
                finishCurrentTransmission();
                numSent++;
            );
            FSMA_Event_Transition(Transmit-EndActive,
                                  msg == endActive,
                                  IDLE,
                EV_WARN << "Disabling communication while transmitting!" << endl;
                finishCurrentTransmission();
                endActiveWhileTransmitting++;
            );
        }
        FSMA_State(RECEIVING)
        {
            EV_INFO << "handling packet with handleWithFsm(): RECEIVING" << endl;
            FSMA_Event_Transition(Receive-Unicast,
                                  isLowerMessage(msg),
                                  ACTIVE,
                sendUp(decapsulate(pkt));
                numReceived++;
                // cancelEvent(endActive);
            );
            FSMA_Event_Transition(Receive-BelowSensitivity,
                                  msg == droppedPacket,
                                  ACTIVE,
            );
            FSMA_Event_Transition(Receive-EndActive,
                                  msg == endActive,
                                  IDLE,
                EV_WARN << "Disabling communication while receiving!" << endl;
                endActiveWhileReceiving++;
            );
        }
    }

    if (fsm.getState() == ACTIVE) {
        // if (isReceiving())
        //    handleWithFsm(mediumStateChange);
        // else
        if (currentTxFrame != nullptr)
            handleWithFsm(currentTxFrame);
        else if (!txQueue->isEmpty()) {
            processUpperPacket();
        }
    }

    if (endSifs) {
        if (isLowerMessage(msg) && pkt->getOwner() == this && (endSifs->getContextPointer() != pkt))
            delete pkt;
    }
    else {
        if (isLowerMessage(msg) && pkt->getOwner() == this)
            delete pkt;
    }
    getDisplayString().setTagArg("t", 0, fsm.getStateName());
}

void LoRaMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == LoRaNodeApp::commActiveChangedSignal) {
        cMessage *activeChange = (value == 1) ? startActive : endActive;
        handleWithFsm(activeChange);
    }
    else if (signalID == IRadio::receptionStateChangedSignal) {
        IRadio::ReceptionState newRadioReceptionState = (IRadio::ReceptionState)value;
        receptionState = newRadioReceptionState;
        handleWithFsm(mediumStateChange);
    }
    else if (signalID == LoRaRadio::droppedPacket) {
        handleWithFsm(droppedPacket);
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            handleWithFsm(endTransmission);
        }
        transmissionState = newRadioTransmissionState;
    }
}

Packet *LoRaMac::encapsulate(Packet *msg)
{
    auto frame = makeShared<LoRaMacFrame>();
    frame->setChunkLength(B(headerLength));
    msg->setArrival(msg->getArrivalModuleId(), msg->getArrivalGateId());
    auto tag = msg->getTag<LoRaTag>();

    frame->setTransmitterAddress(address);
    frame->setLoRaTP(tag->getPower().get());
    frame->setLoRaCF(tag->getCenterFrequency());
    frame->setLoRaSF(tag->getSpreadFactor());
    frame->setLoRaBW(tag->getBandwidth());
    frame->setLoRaCR(tag->getCodeRendundance());
    frame->setSequenceNumber(sequenceNumber);
    frame->setReceiverAddress(MacAddress::BROADCAST_ADDRESS);

    ++sequenceNumber;
    //frame->setLoRaUseHeader(cInfo->getLoRaUseHeader());
    frame->setLoRaUseHeader(tag->getUseHeader());

    msg->insertAtFront(frame);

    return msg;
}

Packet *LoRaMac::decapsulate(Packet *frame)
{
    auto loraHeader = frame->popAtFront<LoRaMacFrame>();
    frame->addTagIfAbsent<MacAddressInd>()->setSrcAddress(loraHeader->getTransmitterAddress());
    frame->addTagIfAbsent<MacAddressInd>()->setDestAddress(loraHeader->getReceiverAddress());
    frame->addTagIfAbsent<InterfaceInd>()->setInterfaceId(networkInterface->getInterfaceId());
//    auto payloadProtocol = ProtocolGroup::ethertype.getProtocol(loraHeader->getNetworkProtocol());
//    frame->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(payloadProtocol);
//    frame->addTagIfAbsent<PacketProtocolTag>()->setProtocol(payloadProtocol);
//
    frame->updateAtFront<LoRaAppPacket>([&] (const Ptr<LoRaAppPacket>& payload) {
        LoRaOptions frameOptions;

        frameOptions.setLoRaTP(loraHeader->getLoRaTP());
        frameOptions.setLoRaCF(loraHeader->getLoRaCF());
        frameOptions.setLoRaSF(loraHeader->getLoRaSF());
        frameOptions.setLoRaBW(loraHeader->getLoRaBW());
        frameOptions.setLoRaCR(loraHeader->getLoRaCR());
        frameOptions.setRSSI(loraHeader->getRSSI());
        frameOptions.setUseHeader(loraHeader->getLoRaUseHeader());
    //    frameOptions.setADRACKReq();
    //    frameOptions.setAppACKReq();

        payload->setOptions(frameOptions);
    });

    return frame;
}

void LoRaMac::invalidateBackoffPeriod()
{
    backoffPeriod = -1;
}

bool LoRaMac::isInvalidBackoffPeriod()
{
    return backoffPeriod == -1;
}

void LoRaMac::generateBackoffPeriod()
{
    ASSERT(0 <= retryCounter && retryCounter <= retryLimit);
    EV << "generating backoff slot number for retry: " << retryCounter << endl;
    int cw = std::min(cwMax, (cwMin + 1) * (1 << retryCounter) - 1);
    int slots = intrand(cw + 1);
    EV << "generated backoff slot number: " << slots << " , cw: " << cw << endl;
    backoffPeriod = slots * slotTime;
    ASSERT(backoffPeriod >= 0);
    EV << "backoff period set to " << backoffPeriod << endl;
}

void LoRaMac::decreaseBackoffPeriod()
{
    simtime_t elapsedBackoffTime = simTime() - endBackoff->getSendingTime();
    backoffPeriod -= ((int)(elapsedBackoffTime / slotTime)) * slotTime;
    ASSERT(backoffPeriod >= 0);
    EV << "backoff period decreased to " << backoffPeriod << endl;
}

void LoRaMac::scheduleBackoffTimer()
{
    EV << "scheduling backoff timer\n";
    if (isInvalidBackoffPeriod())
        generateBackoffPeriod();

    // HACK: Try to handle corner case
    cancelBackoffTimer();
    scheduleAfter(backoffPeriod, endBackoff);
}

void LoRaMac::cancelBackoffTimer()
{
    EV << "canceling backoff timer\n";
    cancelEvent(endBackoff);
}

/****************************************************************
 * Frame sender functions.
 */
void LoRaMac::sendDataFrame(Packet *frameToSend)
{
    EV << "sending Data frame\n";
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto frameCopy = frameToSend->dup();

    //LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
    //ctrl->setSrc(frameCopy->getTransmitterAddress());
    //ctrl->setDest(frameCopy->getReceiverAddress());
    //frameCopy->setControlInfo(ctrl);
    auto macHeader = frameCopy->peekAtFront<LoRaMacFrame>();

    auto macAddressInd = frameCopy->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    //frameCopy->addTag<PacketProtocolTag>()->setProtocol(&Protocol::lora);

    sendDown(frameCopy);
}

void LoRaMac::sendAckFrame()
{
    auto frameToAck = static_cast<Packet *>(endSifs->getContextPointer());
    endSifs->setContextPointer(nullptr);
    auto macHeader = makeShared<CsmaCaMacAckHeader>();
    macHeader->setReceiverAddress(MacAddress(frameToAck->peekAtFront<LoRaMacFrame>()->getTransmitterAddress().getInt()));

    EV << "sending Ack frame\n";
    //auto macHeader = makeShared<CsmaCaMacAckHeader>();
    macHeader->setChunkLength(B(ackLength));
    auto frame = new Packet("CsmaAck");
    frame->insertAtFront(macHeader);
//    frame->addTag<PacketProtocolTag>()->setProtocol(&Protocol::lora);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto macAddressInd = frame->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    sendDown(frame);
}

/****************************************************************
 * Helper functions.
 */
void LoRaMac::finishCurrentTransmission()
{
    deleteCurrentTxFrame();
    resetTransmissionVariables();
}

Packet *LoRaMac::getCurrentTransmission()
{
    ASSERT(currentTxFrame != nullptr);
    return currentTxFrame;
}

void LoRaMac::resetTransmissionVariables()
{
    backoffPeriod = -1;
    retryCounter = 0;
}

bool LoRaMac::isReceiving()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_RECEIVING;
}

bool LoRaMac::isMediumFree()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_IDLE;
}

bool LoRaMac::isAck(const Ptr<const LoRaMacFrame> &frame)
{
    return false;//dynamic_cast<LoRaMacFrame *>(frame);
}

bool LoRaMac::isBroadcast(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getReceiverAddress().isBroadcast();
}

bool LoRaMac::isForUs(const Ptr<const LoRaMacFrame> &frame)
{
    // return frame->getReceiverAddress() == address;
    return true;
}

void LoRaMac::turnOnReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
}

void LoRaMac::turnOffReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

MacAddress LoRaMac::getAddress()
{
    return address;
}

} // namespace inet
