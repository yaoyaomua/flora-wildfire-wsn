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


using namespace inet;

namespace flora {

using namespace inet::power;

class LoRaAppEnergyConsumer: public cSimpleModule, public IEpEnergyConsumer
{
public:
    virtual ~LoRaAppEnergyConsumer();
    virtual void initialize(int stage) override;
    void finish() override;
    virtual void handleMessage(cMessage *message) override;

    virtual void updatePowerConsumption();
    virtual void scheduleIntervalTimer();

    virtual IEnergySource *getEnergySource() const override { return energySource; }
    virtual W getPowerConsumption() const override { return powerConsumption; }

protected:
    int energyConsumerId;
    double totalEnergyConsumed;
    J energyBalance = J(NaN);
    simtime_t lastEnergyBalanceUpdate = -1;
    W lastPowerConsumption = W(0);

    // parameters
    IEpEnergySource *energySource = nullptr;
    cMessage *timer = nullptr;

    // state
    bool isSleeping = false;
    W powerConsumption = W(NaN);
};
}



#endif
/* LORAENERGYMODULES_LORAAPPENERGYCONSUMER_H_ */
