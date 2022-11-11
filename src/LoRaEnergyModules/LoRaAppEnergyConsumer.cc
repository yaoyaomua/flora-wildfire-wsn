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

void LoRaAppEnergyConsumer::initialize(int stage){
    if (stage == INITSTAGE_LOCAL) {
        cModule *appModule = getParentModule()->getSubmodule("app", 0);
        appModule->subscribe(LoRaNodeApp::appModeChangedSignal, this);
        app = check_and_cast<LoRaNodeApp *>(appModule);

        sleepAppPowerConsumption = W(par("sleepAppPowerConsumption"));
        runAppPowerConsumption = W(par("runAppPowerConsumption"));

        totalEnergyConsumed = 0;
        energyBalance = J(0);
        powerConsumption = W(0);
        //energySource.reference(this, "energySourceModule", true);
        energySourceP = check_and_cast<IEpEnergySource *>(getParentModule()->getSubmodule(par("energySourceModule")));
        WATCH(powerConsumption);
    }
    else if (stage == INITSTAGE_POWER)
        energySourceP->addEnergyConsumer(this);

}

void LoRaAppEnergyConsumer::finish()
{
    recordScalar("totalEnergyConsumed", double(totalEnergyConsumed));
}

void LoRaAppEnergyConsumer::receiveSignal(cComponent *source, simsignal_t signal, intval_t value, cObject *details)
{
    Enter_Method("%s", cComponent::getSignalName(signal));

    if (signal == LoRaNodeApp::appModeChangedSignal)
    {
        powerConsumption = getPowerConsumption();
        emit(powerConsumptionChangedSignal, powerConsumption.get());

        simtime_t currentSimulationTime = simTime();
        energyBalance += s((currentSimulationTime - lastEnergyBalanceUpdate).dbl()) * (lastPowerConsumption);
        totalEnergyConsumed = (energyBalance.get());
//        EV_INFO << "Power consumption: " << powerConsumption
//                << " Last power consumption: " << lastPowerConsumption
//                << " Elapsed time (ms): " << ms((currentSimulationTime - lastEnergyBalanceUpdate).dbl())
//                << " Energy Balance delta: " << (s((currentSimulationTime - lastEnergyBalanceUpdate).dbl()) * (lastPowerConsumption)).get()
//                << endl;
        lastEnergyBalanceUpdate = currentSimulationTime;
        lastPowerConsumption = powerConsumption;
    }
    else
        throw cRuntimeError("Unknown signal");
}

W LoRaAppEnergyConsumer::getPowerConsumption() const
{
    LoRaNodeApp::AppMode appMode = app->getAppMode();

    if (appMode == LoRaNodeApp::APP_MODE_SLEEP)
      return sleepAppPowerConsumption;
    else if (appMode == LoRaNodeApp::APP_MODE_RUN)
      return runAppPowerConsumption;

    return powerConsumption;
}

}


