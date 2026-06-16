#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/application/traci/RuleBasedDrivingController.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIColor.h"
#include "veins/veins.h"
#include <exception>
#include <unordered_map>
#include <string>
#include <cmath>

using namespace veins;

Define_Module(veins::TraCIDemo11p);
// This allocates the actual memory for the shared variables
std::map<std::string, int> TraCIDemo11p::laneToIndexMap;
int TraCIDemo11p::nextLaneIndex = 1;

// Structure for the Smart Flip logic
struct FollowerRecord {
    int followerID;
    double updateTime;
};

void TraCIDemo11p::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    frameCounter = 0;
    if (stage == 0) {
        ruleBasedControlEnabled = par("ruleBasedControlEnabled").boolValue();
        ruleDrivingParams.safeTimeHeadway = par("ruleSafeTimeHeadway").doubleValue();
        ruleDrivingParams.minSpatialGap = par("ruleMinSpatialGap").doubleValue();
        ruleDrivingParams.leaderSearchDistance = par("ruleLeaderSearchDistance").doubleValue();
        ruleDrivingParams.controlHorizon = par("ruleControlHorizon").doubleValue();
        ruleDrivingParams.idmA = par("ruleIdmAccel").doubleValue();
        ruleDrivingParams.idmB = par("ruleIdmComfortDecel").doubleValue();
        ruleDrivingParams.kinematicDecel = par("ruleKinematicDecel").doubleValue();
        ruleDrivingParams.maxCmdDecel = par("ruleMaxCmdDecel").doubleValue();
        ruleDrivingParams.ttcThreshold = par("ruleTtcThreshold").doubleValue();
        ruleDrivingParams.idmDelta = par("ruleIdmDelta").doubleValue();
        ruleDrivingParams.steerLaneGain = par("ruleSteerLaneGain").doubleValue();
        ruleDrivingParams.steerMaxRad = par("ruleSteerMaxRad").doubleValue();
        ruleDrivingParams.avoidanceSummaryPath = par("ruleAvoidanceSummaryPath").stdstringValue();
        ruleDrivingParams.safetyDecisionLogPath = par("ruleSafetyDecisionLogPath").stdstringValue();
        ruleDrivingParams.emergencyMinClosingSpeed = par("ruleEmergencyMinClosingSpeed").doubleValue();
        ruleDrivingParams.emergencyTtcThreshold = par("ruleEmergencyTtcThreshold").doubleValue();
        ruleDrivingParams.emergencyGapThreshold = par("ruleEmergencyGapThreshold").doubleValue();
        ruleDrivingParams.brakingSufficientMargin = par("ruleBrakingSufficientMargin").doubleValue();
        ruleDrivingParams.laneChangeTraCIDuration = par("ruleLaneChangeTraCIDuration").doubleValue();
        ruleDrivingParams.emergencyLaneChangeMode = par("ruleEmergencyLaneChangeMode").intValue();
        ruleDrivingParams.defaultLaneChangeMode = par("ruleDefaultLaneChangeMode").intValue();
        ruleSumoSafetyColorEnabled = par("ruleSumoSafetyColorEnabled").boolValue();
        lastRuleDrivingUpdate = 0;
        ruleDrivingEverUpdated = false;
        ruleDrivingSubscribedToSafetyEnd = false;
        ruleSafetyColorActive = false;
        ruleEmergencyActiveLastStep = false;
        ruleSavedLaneChangeModeValid = false;
        ruleHaveDefaultSumoColor = false;
        // =========================================================
        // 1. Link the OMNeT++ signals to the C++ variables
        // =========================================================
        sigVehicleId = registerSignal("sigVehicleId");
        sigFrameId = registerSignal("sigFrameId");
        sigTotalFrames = registerSignal("sigTotalFrames");
        sigGlobalTime = registerSignal("sigGlobalTime");
        sigLocalX = registerSignal("sigLocalX");
        sigLocalY = registerSignal("sigLocalY");
        sigGlobalX = registerSignal("sigGlobalX");
        sigGlobalY = registerSignal("sigGlobalY");
        sigVel = registerSignal("sigVel");
        sigAcc = registerSignal("sigAcc");
        sigPreceding = registerSignal("sigPreceding");
        sigFollowing = registerSignal("sigFollowing");
        sigSpaceHeadway = registerSignal("sigSpaceHeadway");
        sigTimeHeadway = registerSignal("sigTimeHeadway");
        // --> REGISTER NEW SIGNALS HERE <--
        sigVLength = registerSignal("sigVLength");
        sigVWidth  = registerSignal("sigVWidth");
        sigVClass  = registerSignal("sigVClass");
        //sigLaneId = registerSignal("sigLaneId");
        sigLaneIdNumeric = registerSignal("sigLaneIdNumeric");
        nextLaneIndex = 1;
        // =========================================================
        // 2. START THE BEACON TIMER (The missing piece!)
        // =========================================================
        beaconTimer = new cMessage("My Custom Beacon Timer");

        // Start the first beacon at a random time between 0 and 1s to prevent all cars from broadcasting at the exact same millisecond
        simtime_t firstBeaconTime = simTime() + uniform(0, 0.01) + par("beaconInterval").doubleValue();
        scheduleAt(firstBeaconTime, beaconTimer);
    }
    else if (stage == 1) {
        if (ruleBasedControlEnabled) {
            auto* tm = dynamic_cast<TraCIMobility*>(mobility);
            if (tm && tm->getManager()) {
                tm->getManager()->subscribe(TraCIScenarioManager::traciTimestepSafetyEndSignal, this);
                ruleDrivingSubscribedToSafetyEnd = true;
            }
        }
    }
}
void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void TraCIDemo11p::onWSM(BaseFrame1609_4* frame) {
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);

    // 1. SELF-LOG PREVENTION
    if (wsm->getSenderAddress() == myId) return;

    // 2. DATA EXTRACTION
    int senderSumoId = wsm->getVehicle_ID();
    std::string laneId = wsm->getLane_ID();
    if(laneId.empty()) laneId = "none";
    // --> EXTRACTION FOR AI / LDM DECISION MAKING <--
    double msgVLength = wsm->getV_Length();
    double msgVWidth = wsm->getV_Width();
    int msgVClass = wsm->getV_Class();

    // 3. LOG TO CONSOLE
    EV << "WSM RECEIVED: Vehicle " << senderSumoId << " | Prec: " << wsm->getPreceding()
       << " | Foll: " << wsm->getFollowing() << " | Lane: " << laneId << endl;
    EV << "WSM RECEIVED: Vehicle " << senderSumoId
           << " | Class: " << msgVClass
           << " | Dim: " << msgVLength << "x" << msgVWidth
           << " | Lane: " << laneId << endl;
    // 4. VISUAL FEEDBACK
    getParentModule()->bubble(("Recv ID: " + std::to_string(senderSumoId)).c_str());
}
void TraCIDemo11p::onBeacon(cMessage* msg) {
    TraCIDemo11pMessage* traciMsg = new TraCIDemo11pMessage();
    populateWSM(traciMsg);
    // =========================================================
        // 1. CALCULATE FRAME ID & TOTAL FRAMES
        // =========================================================

        // Increment local counter: 1, 2, 3...
        frameCounter++;
        traciMsg->setFrame_ID(frameCounter);

        // Calculate Total Frames:
        // If your sim ends at 100s and beacons are every 0.1s:
        // Total Frames = 100 / 0.1 = 1000
        double beaconInt = par("beaconInterval").doubleValue();
        double simLimit = 100.0; // Set this to your intended simulation end time
        int totalExpected = (int)(simLimit / beaconInt);

        traciMsg->setTotal_Frames(totalExpected);
    // --- 1. INITIALIZE ALL FIELDS (Prevents junk data in vectors) ---

    traciMsg->setLocal_X(0.0);
    traciMsg->setLocal_Y(0.0);
    traciMsg->setGlobal_X(0.0);
    traciMsg->setGlobal_Y(0.0);
    traciMsg->setV_Length(0.0);
    traciMsg->setV_Width(0.0);
    traciMsg->setV_Vel(0.0);
    traciMsg->setV_Acc(0.0);
    traciMsg->setSpace_Headway(-1.0);
    traciMsg->setTime_Headway(-1.0);

    if (!mobility || !traciVehicle) {
        delete traciMsg;
        return;
    }

    try {
    // Helper for ID extraction (UPDATED: Extracts ONLY numbers from ANY SUMO string)
        auto extractId = [](const std::string& str) -> int {
            std::string numStr = "";
            for (char c : str) {
                if (isdigit(c)) numStr += c;
            }
            return numStr.empty() ? -1 : std::stoi(numStr);
        };

        // --- 2. GET SUMO DATA ---
        std::string sumoIdStr = mobility->getExternalId();
        int mySumoId = extractId(sumoIdStr);
        traciMsg->setVehicle_ID(mySumoId);

        // UPDATED: Round the time to the nearest whole number (e.g., 23.0054 -> 23.0)
        // This perfectly aligns all vehicles in your dataset!
        double exactTime = simTime().dbl();
        double cleanTime = std::round(exactTime);
        traciMsg->setGlobal_Time(cleanTime);


    // Correct way to get Global (SUMO) Time:
    // This gets the current time from the TraCI server, which includes any offsets.
    veins::Coord omnetPos = mobility->getPositionAt(simTime());
    veins::TraCIMobility* traciMobility = dynamic_cast<veins::TraCIMobility*>(mobility);
    if (!traciMobility) {
        delete traciMsg;
        return;
    }

    // Convert coordinates using the connection gateway
    veins::TraCICoord rawSumoPos = traciMobility->getManager()->getConnection()->omnet2traci(omnetPos);
    traciMsg->setGlobal_X(rawSumoPos.x);
    traciMsg->setGlobal_Y(rawSumoPos.y);
    traciMsg->setLocal_Y(traciVehicle->getLanePosition());
    traciMsg->setLocal_X(0.0);

    double currentSpeed = traciVehicle->getSpeed();
    traciMsg->setV_Vel(currentSpeed);
    traciMsg->setV_Acc(traciVehicle->getAcceleration());
    traciMsg->setV_Length(traciVehicle->getLength());
    traciMsg->setV_Width(traciVehicle->getWidth());
    traciMsg->setV_Class(1);
    traciMsg->setLane_ID(traciVehicle->getLaneId().c_str());
    // 1. Get the raw string from SUMO
        std::string currentLaneStr = traciVehicle->getLaneId();

        // 2. Check if we've seen this lane before. If not, give it a new number.
        if (laneToIndexMap.find(currentLaneStr) == laneToIndexMap.end()) {
            laneToIndexMap[currentLaneStr] = nextLaneIndex++;

            // This prints a "Key" to the console so you know what the numbers mean
            EV << "LANE MAPPING: " << currentLaneStr << " is now ID " << laneToIndexMap[currentLaneStr] << endl;
        }

        // 3. Get the numeric ID from our map
        int laneNumericId = laneToIndexMap[currentLaneStr];

    // --- 3. CALCULATE HEADWAYS ---
    int bestPreceding = -1;
    int bestFollowing = -1;
    double spaceHeadway = -1;
    double timeHeadway = -1;

    auto leaderInfo = traciVehicle->getLeader(250.0);
    if (!leaderInfo.first.empty()) {
        bestPreceding = extractId(leaderInfo.first);
        spaceHeadway = leaderInfo.second;
        if (currentSpeed > 0.1) {
            timeHeadway = spaceHeadway / currentSpeed;
        }
    }

    // Logic for Following ID tracking
    static std::unordered_map<int, FollowerRecord> globalLeaderToFollowerMap;
    static std::unordered_map<int, int> currentLeaderOf;

    if (mySumoId != -1) {
        if (currentLeaderOf.find(mySumoId) != currentLeaderOf.end()) {
            int oldLeader = currentLeaderOf[mySumoId];
            if (oldLeader != bestPreceding) {
                if (globalLeaderToFollowerMap[oldLeader].followerID == mySumoId) {
                    globalLeaderToFollowerMap[oldLeader].followerID = -1;
                }
            }
        }
        if (bestPreceding != -1) {
            globalLeaderToFollowerMap[bestPreceding] = {mySumoId, simTime().dbl()};
            currentLeaderOf[mySumoId] = bestPreceding;
        }
        if (globalLeaderToFollowerMap.find(mySumoId) != globalLeaderToFollowerMap.end()) {
            FollowerRecord record = globalLeaderToFollowerMap[mySumoId];
            if (record.followerID != -1 && (simTime().dbl() - record.updateTime) < 2.5) {
                bestFollowing = record.followerID;
            }
        }
    }

    // Store in message
    traciMsg->setPreceding(bestPreceding);
    traciMsg->setFollowing(bestFollowing);
    traciMsg->setSpace_Headway(spaceHeadway);
    traciMsg->setTime_Headway(timeHeadway);
    // =========================================================
        // ADD THE LOGGING CODE HERE (Right before Sending)
        // =========================================================
        EV << "--- BEACON DATA [Vehicle " << mySumoId << "] ---" << endl;
        EV << "Global Time: " << traciMsg->getGlobal_Time() << endl;
        EV << "Frame: " << traciMsg->getFrame_ID() << " / " << traciMsg->getTotal_Frames() << endl;
        EV << "Global Pos: (" << traciMsg->getGlobal_X() << ", " << traciMsg->getGlobal_Y() << ")" << endl;
        EV << "Local Pos:  (" << traciMsg->getLocal_X() << ", " << traciMsg->getLocal_Y() << ")" << endl;
        EV << "Dimensions: L=" << traciMsg->getV_Length() << " W=" << traciMsg->getV_Width() << endl;
        EV << "Motion:     Vel=" << traciMsg->getV_Vel() << " Acc=" << traciMsg->getV_Acc() << endl;
        EV << "Headways:   Space=" << traciMsg->getSpace_Headway() << " Time=" << traciMsg->getTime_Headway() << endl;
        EV << "--------------------------------------------" << endl;
    // --- 4. RECORD GROUND TRUTH DATA VIA SIGNALS ---
    // This logs data into the .vec file
    emit(sigVehicleId, (long)traciMsg->getVehicle_ID());
    emit(sigFrameId, (long)traciMsg->getFrame_ID());
    emit(sigTotalFrames, (long)traciMsg->getTotal_Frames());
    emit(sigGlobalTime, traciMsg->getGlobal_Time());
    emit(sigLocalX, traciMsg->getLocal_X());
    emit(sigLocalY, traciMsg->getLocal_Y());
    emit(sigGlobalX, traciMsg->getGlobal_X());
    emit(sigGlobalY, traciMsg->getGlobal_Y());
    emit(sigVel, traciMsg->getV_Vel());
    emit(sigAcc, traciMsg->getV_Acc());
    emit(sigPreceding, (long)traciMsg->getPreceding());
    emit(sigFollowing, (long)traciMsg->getFollowing());
    emit(sigSpaceHeadway, traciMsg->getSpace_Headway());
    emit(sigTimeHeadway, traciMsg->getTime_Headway());
    // Note: Lane_ID is a string and cannot be emitted to a standard numeric vector
    // --> EMIT NEW SIGNALS TO THE DATASET HERE <--
    emit(sigVLength, traciMsg->getV_Length());
    emit(sigVWidth, traciMsg->getV_Width());
    emit(sigVClass, (long)traciMsg->getV_Class());
    emit(sigLaneIdNumeric, (long)laneNumericId);
    //emit(sigLaneId, traciVehicle->getLaneId().c_str());
    // --- 5. SEND MESSAGE OVER THE AIR ---
    traciMsg->setSenderAddress(myId);
    sendDown(traciMsg);
    }
    catch (const std::exception& e) {
        EV_WARN << "TraCIDemo11p::onBeacon: TraCI/data build skipped (vehicle may have left SUMO): " << e.what() << endl;
        delete traciMsg;
        return;
    }
    catch (...) {
        EV_WARN << "TraCIDemo11p::onBeacon: TraCI/data build skipped (non-standard exception)." << endl;
        delete traciMsg;
        return;
    }
}
void TraCIDemo11p::receiveSignal(cComponent* source, simsignal_t signalID, const SimTime& t, cObject* details)
{
    if (signalID == TraCIScenarioManager::traciTimestepSafetyEndSignal) {
        if (ruleBasedControlEnabled) {
            applyRuleBasedDrivingAfterTraCIStep();
        }
    }
}

void TraCIDemo11p::applyRuleBasedDrivingAfterTraCIStep()
{
    if (!ruleBasedControlEnabled || !traci || !traciVehicle || !mobility)
        return;

    auto* tm = dynamic_cast<TraCIMobility*>(mobility);
    if (!tm)
        return;

    ruleDrivingParams.logVehicleId = mobility->getExternalId();

    double dt = ruleDrivingParams.controlHorizon;
    if (ruleDrivingEverUpdated) {
        dt = SIMTIME_DBL(simTime() - lastRuleDrivingUpdate);
        if (dt < 1e-4)
            dt = ruleDrivingParams.controlHorizon;
    }

    try {
        RuleBasedDrivingController::Command cmd = RuleBasedDrivingController::computeCommand(*traci, tm, *traciVehicle, ruleDrivingParams, dt);

        const bool wasEmergency = ruleEmergencyActiveLastStep;

        const bool emergNow = (cmd.safetyAction == RuleBasedDrivingController::SafetyAction::EmergencyBraking
            || cmd.safetyAction == RuleBasedDrivingController::SafetyAction::EmergencyLaneChange);

        std::string collisionOutcome = "-";
        if (emergNow) {
            collisionOutcome = "pending";
        }
        else if (wasEmergency) {
            if (std::isfinite(cmd.gapM) && cmd.gapM > 2.0) {
                collisionOutcome = "yes_cleared_gap";
            }
            else if (std::isfinite(cmd.gapM) && cmd.gapM <= 0.05) {
                collisionOutcome = "no_overlap_or_contact";
            }
            else {
                collisionOutcome = "marginal_or_unknown";
            }
        }

        RuleBasedDrivingController::appendSafetyDecisionLog(ruleDrivingParams, SIMTIME_DBL(simTime()), cmd, collisionOutcome);

        if (!cmd.requestTraCILaneChange && ruleSavedLaneChangeModeValid) {
            traciVehicle->setLaneChangeMode(ruleSavedLaneChangeMode);
            ruleSavedLaneChangeModeValid = false;
        }

        if (cmd.requestTraCILaneChange && cmd.targetLaneIndex >= 0) {
            if (!ruleSavedLaneChangeModeValid) {
                ruleSavedLaneChangeMode = traciVehicle->getLaneChangeMode();
                ruleSavedLaneChangeModeValid = true;
            }
            traciVehicle->setLaneChangeMode(cmd.appliedLaneChangeMode);
            traciVehicle->changeLane(cmd.targetLaneIndex, cmd.laneChangeDuration);
        }

        traciVehicle->setSpeedMode(0);
        traciVehicle->setSpeed(cmd.appliedSpeedMps);

        if (ruleSumoSafetyColorEnabled) {
            if (!ruleHaveDefaultSumoColor) {
                ruleDefaultSumoColor = traciVehicle->getColor();
                ruleHaveDefaultSumoColor = true;
            }
            if (emergNow) {
                if (cmd.safetyAction == RuleBasedDrivingController::SafetyAction::EmergencyLaneChange) {
                    traciVehicle->setColor(TraCIColor(255, 0, 200, 255));
                }
                else {
                    traciVehicle->setColor(TraCIColor(255, 140, 0, 255));
                }
                ruleSafetyColorActive = true;
            }
            else if (ruleSafetyColorActive && ruleHaveDefaultSumoColor) {
                traciVehicle->setColor(ruleDefaultSumoColor);
                ruleSafetyColorActive = false;
            }
        }

        EV_INFO << "[" << mobility->getExternalId() << "] " << RuleBasedDrivingController::formatAccelFtSteerRad(cmd) << std::endl;

        ruleEmergencyActiveLastStep = emergNow;
    }
    catch (const std::exception& e) {
        EV_WARN << "TraCIDemo11p: TraCI rule control skipped for " << mobility->getExternalId()
                << " (vehicle may have left SUMO): " << e.what() << endl;
    }
    catch (...) {
        EV_WARN << "TraCIDemo11p: TraCI rule control skipped for " << mobility->getExternalId() << " (non-standard exception)." << endl;
    }
    lastRuleDrivingUpdate = simTime();
    ruleDrivingEverUpdated = true;
}

void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    if (msg == beaconTimer) {
        onBeacon(msg);
        scheduleAt(simTime() + par("beaconInterval").doubleValue(), beaconTimer);
    }
    else if (dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(msg);
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);

        if (wsm->getSerial() >= 3) {
            stopService();
            delete(wsm);
        } else {
            scheduleAt(simTime() + 1, wsm);
        }
    } else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
            findHost()->getDisplayString().setTagArg("i", 1, "red");
            sentMessage = true;
            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
            populateWSM(wsm);
            wsm->setDemoData(mobility->getRoadId().c_str());
            sendDown(wsm);
        }
    } else {
        lastDroveAt = simTime();
    }
}

void TraCIDemo11p::finish() {
    EV << "--- FINAL LANE DICTIONARY FOR AI ---" << endl;
        for (auto const& [name, id] : laneToIndexMap) {
            EV << "ID " << id << " = " << name << endl;
        }
    if (ruleDrivingSubscribedToSafetyEnd) {
        auto* tm = dynamic_cast<TraCIMobility*>(mobility);
        if (tm && tm->getManager()) {
            tm->getManager()->unsubscribe(TraCIScenarioManager::traciTimestepSafetyEndSignal, this);
        }
        ruleDrivingSubscribedToSafetyEnd = false;
    }
    if (ruleBasedControlEnabled && traciVehicle) {
        try {
            if (ruleSavedLaneChangeModeValid) {
                traciVehicle->setLaneChangeMode(ruleSavedLaneChangeMode);
                ruleSavedLaneChangeModeValid = false;
            }
            if (ruleSafetyColorActive && ruleHaveDefaultSumoColor) {
                traciVehicle->setColor(ruleDefaultSumoColor);
            }
            traciVehicle->setSpeedMode(0xff);
            traciVehicle->setSpeed(-1);
        }
        catch (const std::exception& e) {
            EV_WARN << "TraCIDemo11p::finish: TraCI restore skipped: " << e.what() << endl;
        }
        catch (...) {
            EV_WARN << "TraCIDemo11p::finish: TraCI restore skipped (non-standard exception)." << endl;
        }
    }
    // Look how beautifully clean this is now! No more file-closing crashes.
    if (beaconTimer) {
        cancelAndDelete(beaconTimer);
        beaconTimer = nullptr;
    }
    DemoBaseApplLayer::finish();
}
