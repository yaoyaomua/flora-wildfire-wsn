/*
 * LoRaAppEnergyConsumer.cc
 *
 *  Created on: 10 Nov 2022
 *  Author: sasa
 */

#include "LoRaAppEnergyConsumer.h"

namespace flora {

using namespace inet::power;

Define_Module(LoRaAppEnergyConsumer);

LoRaAppEnergyConsumer::~LoRaAppEnergyConsumer()
{
    cancelAndDelete(timer);
}


void LoRaAppEnergyConsumer::initialize(int stage){
    if (stage == INITSTAGE_LOCAL) {
        const char *energySourceModule = par("energySourceModule");
        energySource = dynamic_cast<IEpEnergySource *>(getModuleByPath(energySourceModule));
        if (!energySource)
            throw cRuntimeError("Energy source module '%s' not found", energySourceModule);

        timer = new cMessage("timer");
        updatePowerConsumption();
        scheduleIntervalTimer();

        totalEnergyConsumed = 0;
        energyBalance = J(0);
    }
    else if (stage == INITSTAGE_POWER)
        energySource->addEnergyConsumer(this);

}

void LoRaAppEnergyConsumer::finish()
{
    recordScalar("totalEnergyConsumed", double(totalEnergyConsumed));
}

void LoRaAppEnergyConsumer::handleMessage(cMessage *message) {
    if (message == timer) {
        isSleeping = !isSleeping;
        updatePowerConsumption();
        scheduleIntervalTimer();
    }
    else
        throw cRuntimeError("Unknown message");
}

void LoRaAppEnergyConsumer::updatePowerConsumption(){
    powerConsumption = isSleeping ? W(0) : W(par("powerConsumption"));
    emit(IEpEnergySource::powerConsumptionChangedSignal, powerConsumption.get());

    simtime_t currentSimulationTime = simTime();
    //if (currentSimulationTime != lastEnergyBalanceUpdate) {
        energyBalance += s((currentSimulationTime - lastEnergyBalanceUpdate).dbl()) * (lastPowerConsumption);
        totalEnergyConsumed = (energyBalance.get());
        lastEnergyBalanceUpdate = currentSimulationTime;
        lastPowerConsumption = powerConsumption;
    //}
}
void LoRaAppEnergyConsumer::scheduleIntervalTimer(){
    scheduleAfter((isSleeping ? par("sleepInterval") : par("consumptionInterval")), timer);
}

}


