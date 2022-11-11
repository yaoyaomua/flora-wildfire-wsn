#include "LoRaAppEnergyConsumer.h"
#include "LoRaPhy/LoRaTransmitter.h"

namespace flora {

using namespace inet::power;

Define_Module(LoRaAppEnergyConsumer);

void LoRaAppEnergyConsumer::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        if (!readConfigurationFile())
            throw cRuntimeError("LoRaAppEnergyConsumer: error in reading the input configuration file");
        sleepAppPowerConsumption = W(par("sleepAppPowerConsumption"));
        runAppPowerConsumption = W(par("runAppPowerConsumption"));
        switchingAppPowerConsumption = W(par("switchingAppPowerConsumption"));

        cModule *appModule = getParentModule()->getSubmodule("app", 0);
        appModule->subscribe(LoRaNodeApp::appModeChangedSignal, this);
        app = check_and_cast<LoRaNodeApp *>(appModule);

        energySource.reference(this, "energySourceModule", true);

        totalEnergyConsumed = 0;
        energyBalance = J(0);
    }
    else if (stage == INITSTAGE_POWER) {
        energySource->addEnergyConsumer(this);
    }
}

void LoRaAppEnergyConsumer::finish()
{
    recordScalar("totalEnergyConsumed", double(totalEnergyConsumed));
}

bool LoRaAppEnergyConsumer::readConfigurationFile()
{
//    cXMLElement *xmlConfig = par("configFile").xmlValue();
//    if (xmlConfig == nullptr)
//        return false;
//    cXMLElementList tagList;
//    cXMLElement *tempTag;
//    const char *str;
//    std::string sstr;    // for easier comparison
//
//    tagList = xmlConfig->getElementsByTagName("receiverReceivingSupplyCurrent");
//    if(tagList.empty()) {
//        throw cRuntimeError("receiverReceivingSupplyCurrent not defined in the configuration file!");
//    }
//    tempTag = tagList.front();
//    str = tempTag->getAttribute("value");
//    receiverReceivingSupplyCurrent = strtod(str, nullptr);
//
//    tagList = xmlConfig->getElementsByTagName("receiverBusySupplyCurrent");
//    if(tagList.empty()) {
//        throw cRuntimeError("receiverBusySupplyCurrent not defined in the configuration file!");
//    }
//    tempTag = tagList.front();
//    str = tempTag->getAttribute("value");
//    receiverBusySupplyCurrent = strtod(str, nullptr);
//
//    tagList = xmlConfig->getElementsByTagName("idleSupplyCurrent");
//    if(tagList.empty()) {
//        throw cRuntimeError("idleSupplyCurrent not defined in the configuration file!");
//    }
//    tempTag = tagList.front();
//    str = tempTag->getAttribute("value");
//    idleSupplyCurrent = strtod(str, nullptr);
//
//    tagList = xmlConfig->getElementsByTagName("supplyVoltage");
//    if(tagList.empty()) {
//        throw cRuntimeError("supplyVoltage not defined in the configuration file!");
//    }
//    tempTag = tagList.front();
//    str = tempTag->getAttribute("value");
//    supplyVoltage = strtod(str, nullptr);
//
//    tagList = xmlConfig->getElementsByTagName("txSupplyCurrents");
//    if(tagList.empty()) {
//        throw cRuntimeError("txSupplyCurrents not defined in the configuration file!");
//    }
//    tempTag = tagList.front();
//    cXMLElementList txSupplyCurrentList = tempTag->getElementsByTagName("txSupplyCurrent");
//    if (txSupplyCurrentList.empty())
//        throw cRuntimeError("No txSupplyCurrent have been defined in txSupplyCurrents!");
//
//    for (cXMLElementList::const_iterator aComb = txSupplyCurrentList.begin(); aComb != txSupplyCurrentList.end(); aComb++) {
//        const char *txPower, *supplyCurrent;
//        if ((*aComb)->getAttribute("txPower") != nullptr)
//            txPower = (*aComb)->getAttribute("txPower");
//        else
//            txPower = "";
//        if ((*aComb)->getAttribute("supplyCurrent") != nullptr)
//            supplyCurrent = (*aComb)->getAttribute("supplyCurrent");
//        else
//            supplyCurrent = "";
//        transmitterTransmittingSupplyCurrent[strtod(txPower, nullptr)] = strtod(supplyCurrent, nullptr);
//    }
    return true;
}

void LoRaAppEnergyConsumer::receiveSignal(cComponent *source, simsignal_t signal, intval_t value, cObject *details)
{
    if (signal == LoRaNodeApp::appModeChangedSignal)
       {
           powerConsumption = getPowerConsumption();
           emit(powerConsumptionChangedSignal, powerConsumption.get());

           simtime_t currentSimulationTime = simTime();
           //if (currentSimulationTime != lastEnergyBalanceUpdate) {
               energyBalance += s((currentSimulationTime - lastEnergyBalanceUpdate).dbl()) * (lastPowerConsumption);
               totalEnergyConsumed = (energyBalance.get());
               lastEnergyBalanceUpdate = currentSimulationTime;
               lastPowerConsumption = powerConsumption;
           //}
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
    else if (appMode == LoRaNodeApp::APP_MODE_SWITCHING)
      return switchingPowerConsumption;

    W powerConsumption = W(0);

    return powerConsumption;
}

}

