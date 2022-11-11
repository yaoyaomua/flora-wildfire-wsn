/*
 * LoRaAppEnergyConsumer.h
 *
 *  Created on: 10 Nov 2022
 *  Author: sasa
 */

#ifndef LORAENERGYMODULES_LORAAPPENERGYCONSUMER_H_
#define LORAENERGYMODULES_LORAAPPENERGYCONSUMER_H_

#include "inet/power/contract/IEpEnergyConsumer.h"
#include "inet/power/contract/IEpEnergySource.h"

#include "LoRaApp/LoRaNodeApp.h"

using namespace inet;

namespace flora {

using namespace inet::power;

class LoRaAppEnergyConsumer: public cSimpleModule, public IEpEnergyConsumer, public cListener
{
public:
    virtual void initialize(int stage) override;
    void finish() override;
    virtual W getPowerConsumption() const override;
    virtual void receiveSignal(cComponent *source, simsignal_t signal, intval_t value, cObject *details) override;
    virtual power::IEnergySource *getEnergySource() const override { return energySourceP; }

protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    int energyConsumerId;
    double totalEnergyConsumed;
    J energyBalance = J(NaN);
    simtime_t lastEnergyBalanceUpdate = -1;

    W lastPowerConsumption = W(0);
    W runAppPowerConsumption;
    W sleepAppPowerConsumption;

    // environment
    opp_component_ptr<LoRaNodeApp> app;
    opp_component_ptr<power::IEpEnergySource> energySourceP;

    // state
    W powerConsumption = W(NaN);
};
}



#endif
/* LORAENERGYMODULES_LORAAPPENERGYCONSUMER_H_ */
