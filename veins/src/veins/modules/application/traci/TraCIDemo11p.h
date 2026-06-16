#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/application/traci/RuleBasedDrivingController.h"
#include "veins/modules/mobility/traci/TraCIColor.h"
#include <map>

namespace veins {

class VEINS_API TraCIDemo11p : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    void finish() override;
    void receiveSignal(cComponent* source, simsignal_t signalID, const SimTime& t, cObject* details) override;

protected:
    // Add these in the 'protected' section of your class
    static std::map<std::string, int> laneToIndexMap;
    static int nextLaneIndex;
    simsignal_t sigLaneIdNumeric; // We will use this numeric signal

    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;
    cMessage* beaconTimer = nullptr;
    int frameCounter = 0;
    // OMNeT++ Signals for High-Speed Dataset Collection (Numeric Data Only)
    simsignal_t sigVehicleId;
    simsignal_t sigFrameId;      // ADD THIS LINE
    simsignal_t sigTotalFrames;  // ADD THIS LINE (if you are using it)
    simsignal_t sigGlobalTime;
    simsignal_t sigLocalX;
    simsignal_t sigLocalY;
    simsignal_t sigGlobalX;
    simsignal_t sigGlobalY;
    simsignal_t sigVel;
    simsignal_t sigAcc;
    simsignal_t sigPreceding;
    simsignal_t sigFollowing;
    simsignal_t sigSpaceHeadway;
    simsignal_t sigTimeHeadway;
    // --> NEW SIGNALS ADDED HERE <--
    simsignal_t sigVLength;
    simsignal_t sigVWidth;
    simsignal_t sigVClass;
    simsignal_t sigLaneId;

    bool ruleBasedControlEnabled = false;
    bool ruleSumoSafetyColorEnabled = true;
    bool ruleDrivingSubscribedToSafetyEnd = false;
    RuleBasedDrivingController::Params ruleDrivingParams;
    simtime_t lastRuleDrivingUpdate = 0;
    bool ruleDrivingEverUpdated = false;
    bool ruleSafetyColorActive = false;
    bool ruleEmergencyActiveLastStep = false;
    bool ruleSavedLaneChangeModeValid = false;
    int32_t ruleSavedLaneChangeMode = 0;
    bool ruleHaveDefaultSumoColor = false;
    TraCIColor ruleDefaultSumoColor{255, 255, 255, 255};

    void applyRuleBasedDrivingAfterTraCIStep();

    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    // This is the function we just wrote the full version for
    virtual void onBeacon(cMessage* msg);

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};

}
