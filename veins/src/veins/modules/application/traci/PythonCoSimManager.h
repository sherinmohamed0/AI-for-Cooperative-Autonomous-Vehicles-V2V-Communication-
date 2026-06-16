#ifndef PYTHON_COSIM_MANAGER_H
#define PYTHON_COSIM_MANAGER_H

#include <omnetpp.h>
#include <cstdint>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "json.hpp"
#include "veins/modules/application/traci/RuleBasedDrivingController.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

using namespace omnetpp;
using json = nlohmann::json;

class PythonCoSimManager : public cSimpleModule, public cListener {
  private:
    int udpSocket = -1;
    struct sockaddr_in serverAddr {};
    cMessage* tickTimer = nullptr;
    veins::TraCICommandInterface* traci = nullptr;
    veins::TraCIScenarioManager* traciScenarioManager = nullptr;

    std::string aiServerHost;
    int aiServerPort = 9999;
    double tickInterval = 0.1;
    double aiResponseTimeoutMs = 5000.0;
    uint64_t tickSeq = 0;
    bool coSimEnabled = true;
    bool ruleBasedFallbackEnabled = true;
    bool safetyShieldEnabled = true;
    bool proactiveLaneEscapeEnabled = true;
    double proactiveLaneEscapeLateralCommand = 2.0;
    double aiLaneChangeThreshold = 1.5;
    double aiLaneChangeUrgentThreshold = 0.3;
    double aiLaneChangeUrgentGapM = 25.0;
    std::string decisionSummaryPath;
    bool decisionSummaryStarted = false;

    std::string destinationTrackingPath;
    std::string vehicleRouteFilePath;
    int closeDestinationEdgeThreshold = 2;
    double destinationTrackingInterval = 1.0;
    int64_t lastDestinationTrackingLogBucket = -1;
    std::ofstream destinationTrackingStream;
    std::map<std::string, std::string> plannedDestinationByVehicle;
    std::map<std::string, std::vector<std::string>> plannedRouteByVehicle;

    struct VehicleDestinationSnapshot {
        std::string edge;
        double lanePosition = -1.0;
        int remainingEdges = -1;
        bool valid = false;
    };
    std::map<std::string, VehicleDestinationSnapshot> lastDestinationTrackingSnapshotByVehicle;

    veins::RuleBasedDrivingController::Params ruleParams;
    std::map<std::string, simtime_t> lastRuleUpdateByVehicle;
    std::map<std::string, int32_t> savedLaneChangeModeByVehicle;
    std::set<std::string> savedLaneChangeModeValid;
    std::map<std::string, double> lastAiTargetSpeedByVehicle;

    struct LeaderContext {
        bool hasLeader = false;
        std::string leaderId;
        double gap = std::numeric_limits<double>::quiet_NaN();
        double vLead = 0.0;
        double closingSpeed = 0.0;
        double ttcSec = std::numeric_limits<double>::infinity();
    };

    LeaderContext queryLeaderContext(veins::TraCICommandInterface::Vehicle& ego) const;

    double computeKinematicSpeedCap(const LeaderContext& lc, double curSpeed) const;

    int applyAiLaneChange(
        veins::TraCICommandInterface::Vehicle& ego,
        double a_x,
        double laneThreshold,
        std::string& laneAction,
        bool applyTraCICommand = true) const;

    double computeProactiveEscapeLateral(veins::TraCICommandInterface::Vehicle& ego, const LeaderContext& lc) const;

    void applyRuleCommand(
        const std::string& vehicleId,
        veins::TraCIMobility* tm,
        const veins::RuleBasedDrivingController::Command& cmd);

    void applySafetyShield(
        const std::string& vehicleId,
        veins::TraCIMobility* tm,
        double proposedSpeed,
        double lateralCommand,
        int aiLaneIndex,
        const std::string& aiLaneAction,
        double a_x,
        double a_y,
        const std::string& decisionSource,
        const std::string& shieldReason,
        double responseTimeMs);

    void applyPostInjectSafetyShield();

    veins::TraCIMobility* findMobilityForVehicle(const std::string& vehicleId) const;

    void appendDecisionSummary(
        double simTimeS,
        const std::string& vehicleId,
        const std::string& source,
        const std::string& decisionType,
        const std::string& actionDescription,
        double targetSpeedMs,
        double lateralCommand,
        int targetLaneIndex,
        const std::string& fallbackReason,
        double responseTimeMs);

    void discardPendingAiResponses();

    bool waitForAiResponse(
        char* buffer,
        size_t bufferSize,
        int& bytesReceived,
        double& responseTimeMs,
        double expectedTimeSec,
        uint64_t expectedTickSeq);

    bool responseMatchesRequestFrame(const json& response, double expectedTimeSec, uint64_t expectedTickSeq) const;

    void applyAiCommand(
        const json& cmd,
        const std::set<std::string>& activeVehicles,
        const std::string& vehicleId,
        veins::TraCIMobility* tm,
        double responseTimeMs);

    void applyRuleBasedFallback(
        const std::string& vehicleId,
        veins::TraCIMobility* tm,
        const std::string& fallbackReason);

    std::string classifyAiDecisionType(double a_x, double a_y) const;
    std::string classifyRuleDecisionType(const veins::RuleBasedDrivingController::Command& cmd) const;
    std::string resolveOutputPath(const std::string& configuredPath) const;

    void loadVehicleRoutesFromFile(const std::string& path);
    void openDestinationTrackingLog();
    void closeDestinationTrackingLog();
    void logVehicleDestinationProgress(const std::string& vehicleId);
    void logDestinationTrackingIfDue();

  protected:
    int numInitStages() const override
    {
        return 2;
    }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage* msg) override;
    virtual void finish() override;
    virtual void finish(cComponent* component, simsignal_t signalID) override;
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, const SimTime& t, cObject* details) override;
};

Define_Module(PythonCoSimManager);
#endif
