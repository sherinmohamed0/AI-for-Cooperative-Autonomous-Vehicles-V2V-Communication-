#include "PythonCoSimManager.h"

#include "veins/base/utils/FindModule.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

using veins::TraCIConstants::LANECHANGE_LEFT;
using veins::TraCIConstants::LANECHANGE_RIGHT;

namespace {

const uint8_t NB_LEFT_AHEAD_BLOCKING = 6;
const uint8_t NB_LEFT_BEHIND_BLOCKING = 4;
const uint8_t NB_RIGHT_AHEAD_BLOCKING = 7;
const uint8_t NB_RIGHT_BEHIND_BLOCKING = 5;

std::string extractXmlAttribute(const std::string& tag, const std::string& attrName)
{
    const std::string key = attrName + "=\"";
    const size_t pos = tag.find(key);
    if (pos == std::string::npos) {
        return std::string();
    }
    const size_t valueStart = pos + key.size();
    const size_t valueEnd = tag.find('"', valueStart);
    if (valueEnd == std::string::npos) {
        return std::string();
    }
    return tag.substr(valueStart, valueEnd - valueStart);
}

std::vector<std::string> splitEdgeList(const std::string& edges)
{
    std::vector<std::string> result;
    std::istringstream iss(edges);
    std::string token;
    while (iss >> token) {
        result.push_back(token);
    }
    return result;
}

std::string trimXmlLine(const std::string& line)
{
    const size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return std::string();
    }
    const size_t end = line.find_last_not_of(" \t\r\n");
    return line.substr(start, end - start + 1);
}

std::string getHostCurrentWorkingDir()
{
#ifdef _WIN32
    char buf[4096];
    if (_getcwd(buf, sizeof(buf))) {
        return std::string(buf);
    }
#else
    char buf[4096];
    if (getcwd(buf, sizeof(buf))) {
        return std::string(buf);
    }
#endif
    return std::string();
}

} // namespace

void PythonCoSimManager::initialize(int stage)
{
    if (stage != 0) {
        if (stage == 1) {
            if (!coSimEnabled) {
                return;
            }
            traciScenarioManager = veins::TraCIScenarioManagerAccess().get();
            if (traciScenarioManager == nullptr) {
                throw cRuntimeError("PythonCoSimManager: TraCIScenarioManager not found");
            }
            traciScenarioManager->subscribe(veins::TraCIScenarioManager::traciTimestepEndSignal, this);
            traciScenarioManager->subscribe(veins::TraCIScenarioManager::traciTimestepSafetyEndSignal, this);
        }
        return;
    }

    coSimEnabled = par("enabled").boolValue();
    aiServerHost = par("aiServerHost").stdstringValue();
    aiServerPort = par("aiServerPort").intValue();
    tickInterval = par("tickInterval").doubleValue();
    aiResponseTimeoutMs = par("aiResponseTimeoutMs").doubleValue();
    ruleBasedFallbackEnabled = par("ruleBasedFallbackEnabled").boolValue();
    safetyShieldEnabled = par("safetyShieldEnabled").boolValue();
    proactiveLaneEscapeEnabled = par("proactiveLaneEscapeEnabled").boolValue();
    proactiveLaneEscapeLateralCommand = par("proactiveLaneEscapeLateralCommand").doubleValue();
    aiLaneChangeThreshold = par("aiLaneChangeThreshold").doubleValue();
    aiLaneChangeUrgentThreshold = par("aiLaneChangeUrgentThreshold").doubleValue();
    aiLaneChangeUrgentGapM = par("aiLaneChangeUrgentGapM").doubleValue();

    if (!coSimEnabled) {
        EV_INFO << "PythonCoSimManager: disabled — no AI or rule-based TraCI control (SUMO default driving)" << endl;
        return;
    }
    decisionSummaryPath = par("decisionSummaryPath").stdstringValue();
    destinationTrackingPath = par("destinationTrackingPath").stdstringValue();
    vehicleRouteFilePath = par("vehicleRouteFilePath").stdstringValue();
    closeDestinationEdgeThreshold = par("closeDestinationEdgeThreshold").intValue();
    destinationTrackingInterval = par("destinationTrackingInterval").doubleValue();
    if (destinationTrackingInterval <= 0) {
        throw cRuntimeError("PythonCoSimManager: destinationTrackingInterval must be > 0");
    }

    if (!vehicleRouteFilePath.empty()) {
        loadVehicleRoutesFromFile(vehicleRouteFilePath);
    }
    openDestinationTrackingLog();

    ruleParams.safeTimeHeadway = par("ruleSafeTimeHeadway").doubleValue();
    ruleParams.minSpatialGap = par("ruleMinSpatialGap").doubleValue();
    ruleParams.leaderSearchDistance = par("ruleLeaderSearchDistance").doubleValue();
    ruleParams.controlHorizon = par("ruleControlHorizon").doubleValue();
    ruleParams.idmA = par("ruleIdmAccel").doubleValue();
    ruleParams.idmB = par("ruleIdmComfortDecel").doubleValue();
    ruleParams.kinematicDecel = par("ruleKinematicDecel").doubleValue();
    ruleParams.maxCmdDecel = par("ruleMaxCmdDecel").doubleValue();
    ruleParams.ttcThreshold = par("ruleTtcThreshold").doubleValue();
    ruleParams.idmDelta = par("ruleIdmDelta").doubleValue();
    ruleParams.emergencyMinClosingSpeed = par("ruleEmergencyMinClosingSpeed").doubleValue();
    ruleParams.emergencyTtcThreshold = par("ruleEmergencyTtcThreshold").doubleValue();
    ruleParams.emergencyGapThreshold = par("ruleEmergencyGapThreshold").doubleValue();
    ruleParams.brakingSufficientMargin = par("ruleBrakingSufficientMargin").doubleValue();
    ruleParams.laneChangeTraCIDuration = par("ruleLaneChangeTraCIDuration").doubleValue();
    ruleParams.emergencyLaneChangeMode = par("ruleEmergencyLaneChangeMode").intValue();
    ruleParams.defaultLaneChangeMode = par("ruleDefaultLaneChangeMode").intValue();
    ruleParams.junctionNearbyRadiusM = par("ruleJunctionNearbyRadiusM").doubleValue();
    ruleParams.junctionMinSafeGapM = par("ruleJunctionMinSafeGapM").doubleValue();
    ruleParams.proactiveLaneChangeGapM = par("ruleProactiveLaneChangeGapM").doubleValue();
    ruleParams.proactiveStoppedLeaderMaxSpeedMps = par("ruleProactiveStoppedLeaderMaxSpeedMps").doubleValue();

    udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpSocket < 0) {
        throw cRuntimeError("PythonCoSimManager: failed to create UDP socket");
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(static_cast<uint16_t>(aiServerPort));
    if (inet_pton(AF_INET, aiServerHost.c_str(), &serverAddr.sin_addr) <= 0) {
        throw cRuntimeError("PythonCoSimManager: invalid aiServerHost \"%s\"", aiServerHost.c_str());
    }

    tickTimer = new cMessage("pythonCoSimTick");
    scheduleAt(simTime() + tickInterval, tickTimer);

    EV_INFO << "PythonCoSimManager: AI server " << aiServerHost << ":" << aiServerPort
            << ", timeout=" << aiResponseTimeoutMs << "ms"
            << ", rule-based fallback=" << (ruleBasedFallbackEnabled ? "on" : "off")
            << ", safety shield=" << (safetyShieldEnabled ? "on" : "off") << endl;
}

std::string PythonCoSimManager::resolveOutputPath(const std::string& configuredPath) const
{
    if (configuredPath.empty()) {
        return configuredPath;
    }
    if (configuredPath[0] == '/') {
        return configuredPath;
    }
#if defined(_WIN32)
    if (configuredPath.length() > 2 && configuredPath[1] == ':' && (configuredPath[2] == '/' || configuredPath[2] == '\\')) {
        return configuredPath;
    }
#endif
    std::string base = getHostCurrentWorkingDir();
    if (!base.empty() && base.back() != '/' && base.back() != '\\') {
        base += '/';
    }
    return base + configuredPath;
}

void PythonCoSimManager::loadVehicleRoutesFromFile(const std::string& path)
{
    const std::string resolvedPath = resolveOutputPath(path);
    std::ifstream in(resolvedPath.c_str());
    if (!in.is_open()) {
        EV_WARN << "PythonCoSimManager: could not open vehicle route file \"" << resolvedPath << "\"" << endl;
        return;
    }

    std::map<std::string, std::vector<std::string>> routeEdgesById;
    std::string pendingVehicleId;
    int loadedTrips = 0;
    int loadedVehicles = 0;

    std::string rawLine;
    while (std::getline(in, rawLine)) {
        const std::string line = trimXmlLine(rawLine);
        if (line.empty()) {
            continue;
        }

        if (line.compare(0, 6, "<trip ") == 0) {
            const std::string vehicleId = extractXmlAttribute(line, "id");
            const std::string destinationEdge = extractXmlAttribute(line, "to");
            if (!vehicleId.empty() && !destinationEdge.empty()) {
                plannedDestinationByVehicle[vehicleId] = destinationEdge;
                ++loadedTrips;
            }
            continue;
        }

        if (line.compare(0, 7, "<route ") == 0) {
            const std::string routeId = extractXmlAttribute(line, "id");
            const std::string edges = extractXmlAttribute(line, "edges");
            if (!routeId.empty() && !edges.empty()) {
                routeEdgesById[routeId] = splitEdgeList(edges);
            }
            continue;
        }

        if (line.compare(0, 9, "<vehicle ") == 0) {
            pendingVehicleId = extractXmlAttribute(line, "id");
            const std::string routeRef = extractXmlAttribute(line, "route");
            if (!pendingVehicleId.empty() && !routeRef.empty()) {
                const auto routeIt = routeEdgesById.find(routeRef);
                if (routeIt != routeEdgesById.end() && !routeIt->second.empty()) {
                    plannedRouteByVehicle[pendingVehicleId] = routeIt->second;
                    plannedDestinationByVehicle[pendingVehicleId] = routeIt->second.back();
                    ++loadedVehicles;
                }
                pendingVehicleId.clear();
            }
            continue;
        }

        if (!pendingVehicleId.empty() && line.compare(0, 7, "<route ") == 0) {
            const std::string edges = extractXmlAttribute(line, "edges");
            if (!edges.empty()) {
                const std::vector<std::string> routeEdges = splitEdgeList(edges);
                plannedRouteByVehicle[pendingVehicleId] = routeEdges;
                if (!routeEdges.empty()) {
                    plannedDestinationByVehicle[pendingVehicleId] = routeEdges.back();
                }
                ++loadedVehicles;
            }
            pendingVehicleId.clear();
            continue;
        }

        if (line.compare(0, 10, "</vehicle>") == 0) {
            pendingVehicleId.clear();
        }
    }

    EV_INFO << "PythonCoSimManager: loaded " << loadedTrips << " trip destination(s) and " << loadedVehicles
            << " explicit vehicle route(s) from \"" << resolvedPath << "\"" << endl;
}

void PythonCoSimManager::openDestinationTrackingLog()
{
    if (destinationTrackingPath.empty() || destinationTrackingStream.is_open()) {
        return;
    }

    const std::string path = resolveOutputPath(destinationTrackingPath);
    destinationTrackingStream.open(path.c_str(), std::ios::out | std::ios::trunc);
    if (!destinationTrackingStream.is_open()) {
        EV_WARN << "PythonCoSimManager: could not open destination tracking log \"" << path << "\"" << endl;
        return;
    }

    destinationTrackingStream << "# Vehicle destination tracking log (comma-separated).\n";
    destinationTrackingStream << "# One row per vehicle per destinationTrackingInterval (default 1s), after each SUMO step in that interval.\n";
    destinationTrackingStream
        << "# sim_time_s,vehicle_id,current_edge,destination_edge,remaining_edges,is_close_to_destination,moved_since_last_log,edges_advanced_since_last_log\n";
    destinationTrackingStream.flush();
}

void PythonCoSimManager::closeDestinationTrackingLog()
{
    if (destinationTrackingStream.is_open()) {
        destinationTrackingStream.flush();
        destinationTrackingStream.close();
    }
}

void PythonCoSimManager::logVehicleDestinationProgress(const std::string& vehicleId)
{
    if (destinationTrackingPath.empty() || traci == nullptr) {
        return;
    }

    try {
        if (!destinationTrackingStream.is_open()) {
            openDestinationTrackingLog();
            if (!destinationTrackingStream.is_open()) {
                return;
            }
        }

        auto ego = traci->vehicle(vehicleId);
        const std::string currentEdge = ego.getRoadId();
        const double lanePosition = ego.getLanePosition();
        const std::list<std::string> plannedEdges = ego.getPlannedRoadIds();

        std::string destinationEdge;
        int remainingEdges = -1;
        bool isCloseToDestination = false;
        bool movedSinceLastLog = false;
        int edgesAdvancedSinceLastLog = 0;

        if (!plannedEdges.empty()) {
            destinationEdge = plannedEdges.back();
            remainingEdges = static_cast<int>(plannedEdges.size());
            isCloseToDestination = remainingEdges <= closeDestinationEdgeThreshold;
        }

        if (destinationEdge.empty()) {
            const auto destIt = plannedDestinationByVehicle.find(vehicleId);
            if (destIt != plannedDestinationByVehicle.end()) {
                destinationEdge = destIt->second;
            }
        }

        if (remainingEdges < 0 && !plannedRouteByVehicle.empty()) {
            const auto routeIt = plannedRouteByVehicle.find(vehicleId);
            if (routeIt != plannedRouteByVehicle.end() && !routeIt->second.empty()) {
                if (destinationEdge.empty()) {
                    destinationEdge = routeIt->second.back();
                }
                for (size_t i = 0; i < routeIt->second.size(); ++i) {
                    if (routeIt->second[i] == currentEdge) {
                        remainingEdges = static_cast<int>(routeIt->second.size() - i);
                        isCloseToDestination = remainingEdges <= closeDestinationEdgeThreshold;
                        break;
                    }
                }
            }
        }

        const auto prevIt = lastDestinationTrackingSnapshotByVehicle.find(vehicleId);
        if (prevIt != lastDestinationTrackingSnapshotByVehicle.end() && prevIt->second.valid) {
            const VehicleDestinationSnapshot& prev = prevIt->second;
            if (currentEdge != prev.edge) {
                movedSinceLastLog = true;
            }
            else if (lanePosition > prev.lanePosition + 1e-3) {
                movedSinceLastLog = true;
            }

            if (prev.remainingEdges >= 0 && remainingEdges >= 0 && remainingEdges < prev.remainingEdges) {
                edgesAdvancedSinceLastLog = prev.remainingEdges - remainingEdges;
                movedSinceLastLog = true;
            }
        }

        VehicleDestinationSnapshot& snapshot = lastDestinationTrackingSnapshotByVehicle[vehicleId];
        snapshot.edge = currentEdge;
        snapshot.lanePosition = lanePosition;
        snapshot.remainingEdges = remainingEdges;
        snapshot.valid = true;

        destinationTrackingStream << std::fixed << std::setprecision(6) << simTime().dbl() << "," << vehicleId << ","
                                  << currentEdge << "," << destinationEdge << "," << remainingEdges << ","
                                  << (isCloseToDestination ? "True" : "False") << ","
                                  << (movedSinceLastLog ? "True" : "False") << "," << edgesAdvancedSinceLastLog << "\n";
        destinationTrackingStream.flush();
    }
    catch (const cException& e) {
        EV_WARN << "PythonCoSimManager: destination tracking skipped for " << vehicleId << ": " << e.what() << endl;
    }
    catch (const std::exception& e) {
        EV_WARN << "PythonCoSimManager: destination tracking skipped for " << vehicleId << ": " << e.what() << endl;
    }
}

void PythonCoSimManager::logDestinationTrackingIfDue()
{
    if (destinationTrackingPath.empty()) {
        return;
    }

    const int64_t logBucket = static_cast<int64_t>(std::floor(simTime().dbl() / destinationTrackingInterval));
    if (logBucket == lastDestinationTrackingLogBucket) {
        return;
    }
    lastDestinationTrackingLogBucket = logBucket;

    if (traciScenarioManager == nullptr) {
        traciScenarioManager = veins::TraCIScenarioManagerAccess().get();
    }
    if (traciScenarioManager == nullptr || !traciScenarioManager->isConnected()) {
        return;
    }
    if (traci == nullptr) {
        traci = traciScenarioManager->getCommandInterface();
    }
    if (traci == nullptr) {
        return;
    }

    if (!destinationTrackingStream.is_open()) {
        openDestinationTrackingLog();
    }

    for (auto it = traciScenarioManager->getManagedHosts().begin(); it != traciScenarioManager->getManagedHosts().end(); ++it) {
        logVehicleDestinationProgress(it->first);
    }
}

void PythonCoSimManager::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
}

void PythonCoSimManager::receiveSignal(cComponent* source, simsignal_t signalID, const SimTime& t, cObject* details)
{
    if (!coSimEnabled) {
        return;
    }
    if (signalID == veins::TraCIScenarioManager::traciTimestepEndSignal) {
        logDestinationTrackingIfDue();
        return;
    }
    if (signalID == veins::TraCIScenarioManager::traciTimestepSafetyEndSignal) {
        applyPostInjectSafetyShield();
    }
}

void PythonCoSimManager::appendDecisionSummary(
    double simTimeS,
    const std::string& vehicleId,
    const std::string& source,
    const std::string& decisionType,
    const std::string& actionDescription,
    double targetSpeedMs,
    double lateralCommand,
    int targetLaneIndex,
    const std::string& fallbackReason,
    double responseTimeMs)
{
    if (decisionSummaryPath.empty()) {
        return;
    }

    const std::string path = resolveOutputPath(decisionSummaryPath);
    const std::ios_base::openmode mode = decisionSummaryStarted ? (std::ios::out | std::ios::app) : (std::ios::out | std::ios::trunc);
    std::ofstream out(path.c_str(), mode);
    if (!out.is_open()) {
        EV_WARN << "PythonCoSimManager: could not open decision summary \"" << path << "\"" << endl;
        return;
    }

    if (!decisionSummaryStarted) {
        out << "# Unified AI / rule-based driving decision log (tab-separated).\n";
        out << "# decision_source: AI | rule_based | safety_shield\n";
        out << "# decision_type: braking | acceleration | speed_change | lane_change | lane_change_and_braking | none\n";
        out << "# sim_time_s\tvehicle_id\tdecision_source\tdecision_type\taction_description\ttarget_speed_ms\tlateral_command\ttarget_lane_index\tfallback_reason\tai_response_time_ms\n";
        decisionSummaryStarted = true;
    }

    const int laneOut = targetLaneIndex >= 0 ? targetLaneIndex : -1;
    out << std::fixed << std::setprecision(6) << simTimeS << "\t" << vehicleId << "\t" << source << "\t" << decisionType << "\t"
        << actionDescription << "\t" << targetSpeedMs << "\t" << lateralCommand << "\t" << laneOut << "\t" << fallbackReason << "\t"
        << responseTimeMs << "\n";
    out.flush();
}

void PythonCoSimManager::discardPendingAiResponses()
{
    char buffer[65536];
    fd_set readfds;
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int discarded = 0;
    while (true) {
        FD_ZERO(&readfds);
        FD_SET(udpSocket, &readfds);
        const int sel = select(udpSocket + 1, &readfds, nullptr, nullptr, &tv);
        if (sel <= 0) {
            break;
        }

        socklen_t serverAddrLen = sizeof(serverAddr);
        const int n = recvfrom(
            udpSocket,
            buffer,
            static_cast<int>(sizeof(buffer) - 1),
            0,
            reinterpret_cast<struct sockaddr*>(&serverAddr),
            &serverAddrLen);
        if (n <= 0) {
            break;
        }
        ++discarded;
    }

    if (discarded > 0) {
        EV_INFO << "PythonCoSimManager: discarded " << discarded << " stale AI UDP packet(s)" << endl;
    }
}

bool PythonCoSimManager::responseMatchesRequestFrame(
    const json& response,
    double expectedTimeSec,
    uint64_t expectedTickSeq) const
{
    if (response.contains("Tick_seq") && response["Tick_seq"].is_number()) {
        return response["Tick_seq"].get<uint64_t>() == expectedTickSeq;
    }

    if (response.contains("Time_sec") && response["Time_sec"].is_number()) {
        return std::abs(response["Time_sec"].get<double>() - expectedTimeSec) < 1e-6;
    }

    return false;
}

bool PythonCoSimManager::waitForAiResponse(
    char* buffer,
    size_t bufferSize,
    int& bytesReceived,
    double& responseTimeMs,
    double expectedTimeSec,
    uint64_t expectedTickSeq)
{
    bytesReceived = 0;
    responseTimeMs = 0.0;

    const auto waitStart = std::chrono::steady_clock::now();
    const auto waitDeadline = waitStart + std::chrono::duration<double, std::milli>(aiResponseTimeoutMs);
    int discardedLate = 0;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= waitDeadline) {
            break;
        }

        const auto remainingMs = std::chrono::duration_cast<std::chrono::milliseconds>(waitDeadline - now).count();

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(udpSocket, &readfds);

        struct timeval tv {};
        tv.tv_sec = remainingMs / 1000;
        tv.tv_usec = (remainingMs % 1000) * 1000;

        const int sel = select(udpSocket + 1, &readfds, nullptr, nullptr, &tv);
        if (sel <= 0) {
            break;
        }

        socklen_t serverAddrLen = sizeof(serverAddr);
        const int n = recvfrom(
            udpSocket,
            buffer,
            static_cast<int>(bufferSize - 1),
            0,
            reinterpret_cast<struct sockaddr*>(&serverAddr),
            &serverAddrLen);
        if (n <= 0) {
            break;
        }

        buffer[n] = '\0';
        try {
            json response = json::parse(buffer);
            if (responseMatchesRequestFrame(response, expectedTimeSec, expectedTickSeq)) {
                bytesReceived = n;
                const auto waitEnd = std::chrono::steady_clock::now();
                responseTimeMs = std::chrono::duration<double, std::milli>(waitEnd - waitStart).count();
                return true;
            }

            ++discardedLate;
            EV_WARN << "PythonCoSimManager: ignoring late AI reply for tick_seq="
                    << response.value("Tick_seq", static_cast<uint64_t>(0))
                    << " time_sec=" << response.value("Time_sec", 0.0)
                    << " (expected tick_seq=" << expectedTickSeq << " time_sec=" << expectedTimeSec << ")" << endl;
        }
        catch (const json::exception& e) {
            ++discardedLate;
            EV_WARN << "PythonCoSimManager: ignoring malformed AI UDP packet: " << e.what() << endl;
        }
    }

    if (discardedLate > 0) {
        EV_WARN << "PythonCoSimManager: discarded " << discardedLate
                << " late AI repl(ies) for tick_seq=" << expectedTickSeq << " time_sec=" << expectedTimeSec << endl;
    }

    return false;
}

std::string PythonCoSimManager::classifyAiDecisionType(double a_x, double a_y) const
{
    const bool lane = std::abs(a_x) > 1.5;
    const bool brake = a_y < -0.1;
    const bool accel = a_y > 0.1;

    if (lane && brake) {
        return "lane_change_and_braking";
    }
    if (lane) {
        return "lane_change";
    }
    if (brake) {
        return "braking";
    }
    if (accel) {
        return "acceleration";
    }
    if (std::abs(a_y) > 1e-6) {
        return "speed_change";
    }
    return "none";
}

std::string PythonCoSimManager::classifyRuleDecisionType(const veins::RuleBasedDrivingController::Command& cmd) const
{
    switch (cmd.safetyAction) {
    case veins::RuleBasedDrivingController::SafetyAction::EmergencyLaneChange:
        return "lane_change_and_braking";
    case veins::RuleBasedDrivingController::SafetyAction::EmergencyBraking:
        return "braking";
    case veins::RuleBasedDrivingController::SafetyAction::LaneFollow:
        if (cmd.requestTraCILaneChange) {
            return "lane_change";
        }
        if (cmd.accelMps2 < -0.1) {
            return "braking";
        }
        if (cmd.accelMps2 > 0.1) {
            return "acceleration";
        }
        return "speed_change";
    default:
        return "none";
    }
}

veins::TraCIMobility* PythonCoSimManager::findMobilityForVehicle(const std::string& vehicleId) const
{
    if (traciScenarioManager == nullptr) {
        return nullptr;
    }
    const auto& hosts = traciScenarioManager->getManagedHosts();
    const auto it = hosts.find(vehicleId);
    if (it == hosts.end()) {
        return nullptr;
    }
    return veins::FindModule<veins::TraCIMobility*>::findSubModule(it->second);
}

PythonCoSimManager::LeaderContext PythonCoSimManager::queryLeaderContext(veins::TraCICommandInterface::Vehicle& ego) const
{
    LeaderContext lc;
    if (traci == nullptr) {
        return lc;
    }

    try {
        const auto leaderInfo = ego.getLeader(ruleParams.leaderSearchDistance);
        if (leaderInfo.first.empty() || !std::isfinite(leaderInfo.second)) {
            return lc;
        }

        lc.hasLeader = true;
        lc.leaderId = leaderInfo.first;
        lc.gap = leaderInfo.second;
        lc.vLead = std::max(0.0, traci->vehicle(lc.leaderId).getSpeed());
        const double vEgo = std::max(0.0, ego.getSpeed());
        lc.closingSpeed = vEgo - lc.vLead;
        if (lc.closingSpeed > ruleParams.emergencyMinClosingSpeed) {
            lc.ttcSec = lc.gap / lc.closingSpeed;
        }
    }
    catch (const cException&) {
        lc = LeaderContext();
    }
    catch (const std::exception&) {
        lc = LeaderContext();
    }

    return lc;
}

double PythonCoSimManager::computeKinematicSpeedCap(const LeaderContext& lc, double curSpeed) const
{
    if (!lc.hasLeader || !std::isfinite(lc.gap) || lc.gap <= 0.0) {
        return curSpeed;
    }

    const double maxDecel = std::max(0.1, ruleParams.maxCmdDecel);
    const double minGap = std::max(0.5, ruleParams.minSpatialGap);
    const double usable = std::max(0.0, lc.gap - minGap);

    if (lc.closingSpeed <= 0.05) {
        return curSpeed;
    }

    double vStop = lc.vLead + std::sqrt(std::max(0.0, 2.0 * maxDecel * usable));
    vStop = std::min(vStop, curSpeed);

    if (lc.gap < minGap + 1.0) {
        vStop = 0.0;
    }

    return std::max(0.0, vStop);
}

int PythonCoSimManager::applyAiLaneChange(
    veins::TraCICommandInterface::Vehicle& ego,
    double a_x,
    double laneThreshold,
    std::string& laneAction,
    bool applyTraCICommand) const
{
    int targetLaneIndex = -1;
    laneAction = "hold_lane";

    if (traci == nullptr || std::abs(a_x) <= laneThreshold) {
        return targetLaneIndex;
    }

    const int curLane = ego.getLaneIndex();
    const std::string laneId = ego.getLaneId();
    const std::string edgeId = traci->lane(laneId).getRoadId();
    const int nLanes = traci->road(edgeId).getLaneNumber();

    auto neighborsClear = [&](uint8_t aheadMode, uint8_t behindMode) -> bool {
        try {
            return ego.getNeighborIds(aheadMode).empty() && ego.getNeighborIds(behindMode).empty();
        }
        catch (const cException&) {
            return false;
        }
        catch (const std::exception&) {
            return false;
        }
    };

    if (a_x > laneThreshold) {
        if (curLane + 1 < nLanes) {
            if (neighborsClear(NB_LEFT_AHEAD_BLOCKING, NB_LEFT_BEHIND_BLOCKING)) {
                targetLaneIndex = curLane + 1;
                if (applyTraCICommand) {
                    ego.setLaneChangeMode(ruleParams.defaultLaneChangeMode);
                    ego.changeLane(targetLaneIndex, 0.5);
                }
                laneAction = "lane_change_left";
            }
            else {
                laneAction = "change_left_blocked_traffic";
            }
        }
        else {
            laneAction = "change_left_blocked_boundary";
        }
    }
    else if (a_x < -laneThreshold) {
        if (curLane - 1 >= 0) {
            if (neighborsClear(NB_RIGHT_AHEAD_BLOCKING, NB_RIGHT_BEHIND_BLOCKING)) {
                targetLaneIndex = curLane - 1;
                if (applyTraCICommand) {
                    ego.setLaneChangeMode(ruleParams.defaultLaneChangeMode);
                    ego.changeLane(targetLaneIndex, 0.5);
                }
                laneAction = "lane_change_right";
            }
            else {
                laneAction = "change_right_blocked_traffic";
            }
        }
        else {
            laneAction = "change_right_blocked_boundary";
        }
    }

    return targetLaneIndex;
}

double PythonCoSimManager::computeProactiveEscapeLateral(veins::TraCICommandInterface::Vehicle& ego, const LeaderContext& lc) const
{
    if (!proactiveLaneEscapeEnabled || traci == nullptr || !lc.hasLeader || !std::isfinite(lc.gap)) {
        return 0.0;
    }

    const double s0 = std::max(0.5, ruleParams.minSpatialGap);
    if (lc.vLead > ruleParams.proactiveStoppedLeaderMaxSpeedMps) {
        return 0.0;
    }
    if (lc.gap <= s0 || lc.gap >= ruleParams.proactiveLaneChangeGapM) {
        return 0.0;
    }
    if (lc.closingSpeed <= ruleParams.emergencyMinClosingSpeed) {
        return 0.0;
    }

    std::string probeAction;
    const double lateral = proactiveLaneEscapeLateralCommand;

    if (applyAiLaneChange(ego, lateral, 0.0, probeAction, false) >= 0) {
        return lateral;
    }
    if (applyAiLaneChange(ego, -lateral, 0.0, probeAction, false) >= 0) {
        return -lateral;
    }

    return 0.0;
}

void PythonCoSimManager::applyRuleCommand(
    const std::string& vehicleId,
    veins::TraCIMobility* tm,
    const veins::RuleBasedDrivingController::Command& cmd)
{
    auto ego = traci->vehicle(vehicleId);

    if (!cmd.requestTraCILaneChange && savedLaneChangeModeValid.count(vehicleId) > 0) {
        ego.setLaneChangeMode(savedLaneChangeModeByVehicle[vehicleId]);
        savedLaneChangeModeValid.erase(vehicleId);
    }

    if (cmd.requestTraCILaneChange && cmd.targetLaneIndex >= 0) {
        if (savedLaneChangeModeValid.count(vehicleId) == 0) {
            savedLaneChangeModeByVehicle[vehicleId] = ego.getLaneChangeMode();
            savedLaneChangeModeValid.insert(vehicleId);
        }
        ego.setLaneChangeMode(cmd.appliedLaneChangeMode);
        ego.changeLane(cmd.targetLaneIndex, cmd.laneChangeDuration);
    }

    ego.setSpeedMode(0);
    ego.setSpeed(cmd.appliedSpeedMps);
}

void PythonCoSimManager::applySafetyShield(
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
    double responseTimeMs)
{
    if (traci == nullptr) {
        return;
    }

    try {
        auto ego = traci->vehicle(vehicleId);
        ruleParams.logVehicleId = vehicleId;

        double dt = ruleParams.controlHorizon;
        const auto it = lastRuleUpdateByVehicle.find(vehicleId);
        if (it != lastRuleUpdateByVehicle.end()) {
            dt = SIMTIME_DBL(simTime() - it->second);
            if (dt < 1e-4) {
                dt = ruleParams.controlHorizon;
            }
        }

        const veins::RuleBasedDrivingController::Command ruleCmd =
            veins::RuleBasedDrivingController::computeCommand(*traci, tm, ego, ruleParams, dt);

        double finalSpeed = proposedSpeed;
        std::string reason = shieldReason;
        std::string source = decisionSource;
        int appliedLaneIndex = aiLaneIndex;
        std::string laneAction = aiLaneAction;

        const bool emergency = ruleCmd.safetyAction == veins::RuleBasedDrivingController::SafetyAction::EmergencyBraking
            || ruleCmd.safetyAction == veins::RuleBasedDrivingController::SafetyAction::EmergencyLaneChange;

        if (emergency) {
            finalSpeed = ruleCmd.appliedSpeedMps;
            reason = "safety_emergency";
            source = "safety_shield";
        }
        else if (ruleCmd.appliedSpeedMps < proposedSpeed - 0.05) {
            finalSpeed = ruleCmd.appliedSpeedMps;
            if (reason == "NULL") {
                reason = "safety_capped";
            }
        }

        ego.setSpeedMode(0);
        ego.setSpeed(finalSpeed);

        if (ruleCmd.requestTraCILaneChange && ruleCmd.targetLaneIndex >= 0) {
            applyRuleCommand(vehicleId, tm, ruleCmd);
            appliedLaneIndex = ruleCmd.targetLaneIndex;
            laneAction = "emergency_lane_change";
            source = "safety_shield";
            if (ruleCmd.dangerDescription.find("proactive_lane_escape") != std::string::npos) {
                reason = "proactive_lane_escape";
            }
            else {
                reason = "safety_emergency_lane_change";
            }
        }

        std::ostringstream desc;
        if (source == "safety_shield") {
            desc << veins::RuleBasedDrivingController::safetyActionLabel(ruleCmd.safetyAction);
            if (!ruleCmd.dangerDescription.empty()) {
                desc << " (" << ruleCmd.dangerDescription << ")";
            }
            desc << " v=" << std::fixed << std::setprecision(3) << finalSpeed;
            if (!laneAction.empty()) {
                desc << " " << laneAction;
            }
        }
        else {
            desc << "AI a_y=" << std::fixed << std::setprecision(3) << a_y << " a_x=" << a_x << " -> v=" << finalSpeed << " " << laneAction;
            if (reason != "NULL") {
                desc << " [" << reason << "]";
            }
        }

        const std::string decisionType = (source == "safety_shield") ? classifyRuleDecisionType(ruleCmd)
                                                                   : classifyAiDecisionType(a_x, a_y);

        appendDecisionSummary(
            simTime().dbl(),
            vehicleId,
            source,
            decisionType,
            desc.str(),
            finalSpeed,
            lateralCommand,
            appliedLaneIndex,
            reason,
            responseTimeMs);

        lastRuleUpdateByVehicle[vehicleId] = simTime();
    }
    catch (const cException& e) {
        EV_WARN << "PythonCoSimManager: safety shield failed for " << vehicleId << ": " << e.what() << endl;
    }
    catch (const std::exception& e) {
        EV_WARN << "PythonCoSimManager: safety shield failed for " << vehicleId << ": " << e.what() << endl;
    }
}

void PythonCoSimManager::applyPostInjectSafetyShield()
{
    if (!safetyShieldEnabled || !ruleBasedFallbackEnabled || traci == nullptr) {
        return;
    }

    if (traciScenarioManager == nullptr) {
        traciScenarioManager = veins::TraCIScenarioManagerAccess().get();
    }
    if (traciScenarioManager == nullptr || !traciScenarioManager->isConnected()) {
        return;
    }

    for (auto it = traciScenarioManager->getManagedHosts().begin(); it != traciScenarioManager->getManagedHosts().end(); ++it) {
        const std::string& vid = it->first;
        veins::TraCIMobility* tm = veins::FindModule<veins::TraCIMobility*>::findSubModule(it->second);

        try {
            auto ego = traci->vehicle(vid);
            const double curSpeed = ego.getSpeed();

            double proposed = curSpeed;
            const auto speedIt = lastAiTargetSpeedByVehicle.find(vid);
            if (speedIt != lastAiTargetSpeedByVehicle.end()) {
                proposed = speedIt->second;
            }

            ruleParams.logVehicleId = vid;
            const veins::RuleBasedDrivingController::Command ruleCmd =
                veins::RuleBasedDrivingController::computeCommand(*traci, tm, ego, ruleParams, tickInterval);

            const bool emergency = ruleCmd.safetyAction == veins::RuleBasedDrivingController::SafetyAction::EmergencyBraking
                || ruleCmd.safetyAction == veins::RuleBasedDrivingController::SafetyAction::EmergencyLaneChange;
            const bool needsCap = ruleCmd.appliedSpeedMps < proposed - 0.05;
            const bool injectorOverride = curSpeed > proposed + 0.5;

            if (!emergency && !needsCap && !injectorOverride && !ruleCmd.requestTraCILaneChange) {
                continue;
            }

            applySafetyShield(vid, tm, proposed, 0.0, -1, "hold_lane", 0.0, 0.0, "safety_shield", "post_inject_shield", 0.0);
        }
        catch (const cException&) {
            continue;
        }
        catch (const std::exception&) {
            continue;
        }
    }
}

void PythonCoSimManager::applyAiCommand(
    const json& cmd,
    const std::set<std::string>& activeVehicles,
    const std::string& vehicleId,
    veins::TraCIMobility* tm,
    double responseTimeMs)
{
    if (!coSimEnabled) {
        return;
    }
    if (activeVehicles.find(vehicleId) == activeVehicles.end()) {
        EV_WARN << "PythonCoSimManager: AI command for inactive vehicle '" << vehicleId << "' skipped" << endl;
        return;
    }

    const double a_y = cmd.value("a_y", 0.0);
    const double a_x = cmd.value("a_x", 0.0);

    auto ego = traci->vehicle(vehicleId);
    ego.setSpeedMode(0);
    ego.setLaneChangeMode(0);
    const double curSpeed = ego.getSpeed();

    double aiTarget = std::max(0.0, curSpeed + (a_y * tickInterval));

    const LeaderContext lc = queryLeaderContext(ego);
    if (lc.hasLeader) {
        aiTarget = std::min(aiTarget, computeKinematicSpeedCap(lc, curSpeed));
        if (lc.closingSpeed > ruleParams.emergencyMinClosingSpeed
            && std::isfinite(lc.gap)
            && lc.gap < ruleParams.minSpatialGap + 1.0) {
            aiTarget = 0.0;
        }
    }

    lastAiTargetSpeedByVehicle[vehicleId] = aiTarget;

    double effectiveAx = a_x;
    const double escapeAx = computeProactiveEscapeLateral(ego, lc);
    if (std::abs(escapeAx) > std::abs(effectiveAx)) {
        effectiveAx = escapeAx;
    }

    double laneThreshold = aiLaneChangeThreshold;
    if (lc.hasLeader && std::isfinite(lc.gap) && lc.gap < aiLaneChangeUrgentGapM && lc.closingSpeed > ruleParams.emergencyMinClosingSpeed) {
        laneThreshold = aiLaneChangeUrgentThreshold;
    }
    if (std::abs(escapeAx) > 1e-6) {
        laneThreshold = 0.0;
    }

    std::string laneAction;
    const int targetLaneIndex = applyAiLaneChange(ego, effectiveAx, laneThreshold, laneAction);

    if (safetyShieldEnabled && ruleBasedFallbackEnabled) {
        applySafetyShield(
            vehicleId,
            tm,
            aiTarget,
            effectiveAx,
            targetLaneIndex,
            laneAction,
            effectiveAx,
            a_y,
            "AI",
            escapeAx != 0.0 ? "proactive_lane_escape" : "NULL",
            responseTimeMs);
    }
    else {
        ego.setSpeed(aiTarget);

        std::ostringstream desc;
        desc << "AI a_y=" << std::fixed << std::setprecision(3) << a_y << " a_x=" << effectiveAx << " -> v=" << aiTarget << " " << laneAction;

        appendDecisionSummary(
            simTime().dbl(),
            vehicleId,
            "AI",
            classifyAiDecisionType(a_x, a_y),
            desc.str(),
            aiTarget,
            a_x,
            targetLaneIndex,
            "NULL",
            responseTimeMs);
    }
}

void PythonCoSimManager::applyRuleBasedFallback(
    const std::string& vehicleId,
    veins::TraCIMobility* tm,
    const std::string& fallbackReason)
{
    if (!coSimEnabled || !ruleBasedFallbackEnabled || traci == nullptr || tm == nullptr) {
        return;
    }

    try {
        auto ego = traci->vehicle(vehicleId);
        ruleParams.logVehicleId = vehicleId;

        double dt = ruleParams.controlHorizon;
        const auto it = lastRuleUpdateByVehicle.find(vehicleId);
        if (it != lastRuleUpdateByVehicle.end()) {
            dt = SIMTIME_DBL(simTime() - it->second);
            if (dt < 1e-4) {
                dt = ruleParams.controlHorizon;
            }
        }

        const veins::RuleBasedDrivingController::Command cmd =
            veins::RuleBasedDrivingController::computeCommand(*traci, tm, ego, ruleParams, dt);

        applyRuleCommand(vehicleId, tm, cmd);

        const int appliedLaneIndex = cmd.requestTraCILaneChange ? cmd.targetLaneIndex : -1;

        std::ostringstream desc;
        desc << veins::RuleBasedDrivingController::safetyActionLabel(cmd.safetyAction);
        if (!cmd.dangerDescription.empty()) {
            desc << " (" << cmd.dangerDescription << ")";
        }
        desc << " v_target=" << std::fixed << std::setprecision(3) << cmd.appliedSpeedMps;
        if (appliedLaneIndex >= 0) {
            desc << " lane->" << appliedLaneIndex;
        }

        appendDecisionSummary(
            simTime().dbl(),
            vehicleId,
            "rule_based",
            classifyRuleDecisionType(cmd),
            desc.str(),
            cmd.appliedSpeedMps,
            cmd.steerRad,
            appliedLaneIndex,
            fallbackReason,
            0.0);

        lastRuleUpdateByVehicle[vehicleId] = simTime();
    }
    catch (const cException& e) {
        EV_WARN << "PythonCoSimManager: rule-based fallback failed for " << vehicleId << ": " << e.what() << endl;
    }
    catch (const std::exception& e) {
        EV_WARN << "PythonCoSimManager: rule-based fallback failed for " << vehicleId << ": " << e.what() << endl;
    }
}

void PythonCoSimManager::handleMessage(cMessage* msg)
{
    if (!coSimEnabled) {
        delete msg;
        return;
    }

    if (msg != tickTimer) {
        delete msg;
        return;
    }

    cModule* managerModule = getParentModule()->getSubmodule("manager");
    traciScenarioManager = check_and_cast<veins::TraCIScenarioManager*>(managerModule);

    if (traci == nullptr) {
        traci = traciScenarioManager->getCommandInterface();
    }
    if (traci == nullptr || !traciScenarioManager->isConnected()) {
        scheduleAt(simTime() + tickInterval, tickTimer);
        return;
    }

    json payload;
    const double requestTimeSec = simTime().dbl();
    const uint64_t requestTickSeq = ++tickSeq;
    payload["Time_sec"] = requestTimeSec;
    payload["Tick_seq"] = requestTickSeq;
    payload["Vehicles"] = json::array();

    discardPendingAiResponses();

    std::set<std::string> activeVehicles;
    std::map<std::string, veins::TraCIMobility*> mobilityByVehicle;
    for (auto it = traciScenarioManager->getManagedHosts().begin(); it != traciScenarioManager->getManagedHosts().end(); ++it) {
        activeVehicles.insert(it->first);
        auto* hostMod = it->second;
        auto* tm = veins::FindModule<veins::TraCIMobility*>::findSubModule(hostMod);
        if (tm) {
            mobilityByVehicle[it->first] = tm;
        }
    }

    for (const auto& vid : activeVehicles) {
        auto veh = traci->vehicle(vid);
        const veins::Coord pos = veh.getPosition();
        json v;
        v["Car_ID"] = std::stod(vid);
        // Lane position (m along current edge) — must match TraCIDemo11p / CS-LSTM training data.
        v["sigLocalY"] = veh.getLanePosition();
        v["sigGlobalX"] = pos.x;
        v["sigGlobalY"] = pos.y;
        v["sigVel"] = veh.getSpeed();
        v["Lane_ID"] = std::to_string(veh.getLaneIndex());
        v["Road_ID"] = veh.getRoadId();

        try {
            const auto leaderInfo = veh.getLeader(ruleParams.leaderSearchDistance);
            if (!leaderInfo.first.empty() && std::isfinite(leaderInfo.second)) {
                v["Leader_ID"] = leaderInfo.first;
                v["Leader_Gap_m"] = leaderInfo.second;
                const double vEgo = veh.getSpeed();
                const double vLead = std::max(0.0, traci->vehicle(leaderInfo.first).getSpeed());
                v["Leader_Speed_ms"] = vLead;
                const double closing = vEgo - vLead;
                v["Closing_Speed_ms"] = closing;
                if (closing > ruleParams.emergencyMinClosingSpeed) {
                    v["TTC_s"] = leaderInfo.second / closing;
                }
                else {
                    v["TTC_s"] = nullptr;
                }
            }
        }
        catch (const cException&) {
        }
        catch (const std::exception&) {
        }

        payload["Vehicles"].push_back(v);
    }

    const std::string msgStr = payload.dump();
    const int bytesSent = sendto(
        udpSocket,
        msgStr.c_str(),
        static_cast<int>(msgStr.length()),
        0,
        reinterpret_cast<struct sockaddr*>(&serverAddr),
        sizeof(serverAddr));

    if (bytesSent == -1) {
        EV_WARN << "PythonCoSimManager: UDP send failed, errno=" << errno << endl;
        if (ruleBasedFallbackEnabled) {
            for (const auto& vid : activeVehicles) {
                applyRuleBasedFallback(vid, mobilityByVehicle[vid], "ai_send_failed");
            }
        }
        scheduleAt(simTime() + tickInterval, tickTimer);
        return;
    }

    char buffer[65536];
    int bytesReceived = 0;
    double responseTimeMs = 0.0;
    const bool gotResponse = waitForAiResponse(
        buffer,
        sizeof(buffer),
        bytesReceived,
        responseTimeMs,
        requestTimeSec,
        requestTickSeq);

    if (!gotResponse) {
        discardPendingAiResponses();
    }

    std::set<std::string> aiControlledVehicles;

    if (!gotResponse) {
        EV_WARN << "PythonCoSimManager: AI response timeout (" << aiResponseTimeoutMs << "ms) at t=" << simTime()
                << " tick_seq=" << requestTickSeq << endl;
        if (ruleBasedFallbackEnabled) {
            for (const auto& vid : activeVehicles) {
                applyRuleBasedFallback(vid, mobilityByVehicle[vid], "ai_timeout");
            }
        }
    }
    else {
        buffer[bytesReceived] = '\0';
        try {
            json response = json::parse(buffer);
            const bool aiReady = response.value("ai_ready", simTime().dbl() >= 1.6);
            if (response.contains("Commands") && response["Commands"].is_array()) {
                for (auto& cmd : response["Commands"]) {
                    std::string vid;
                    if (cmd["Car_ID"].is_number()) {
                        vid = std::to_string(static_cast<int>(cmd["Car_ID"].get<double>()));
                    }
                    else if (cmd["Car_ID"].is_string()) {
                        std::string rawId = cmd["Car_ID"].get<std::string>();
                        const size_t dotPos = rawId.find('.');
                        vid = (dotPos != std::string::npos) ? rawId.substr(0, dotPos) : rawId;
                    }
                    else {
                        continue;
                    }

                    applyAiCommand(cmd, activeVehicles, vid, mobilityByVehicle[vid], responseTimeMs);
                    aiControlledVehicles.insert(vid);
                }
            }

            if (ruleBasedFallbackEnabled && aiReady) {
                for (const auto& vid : activeVehicles) {
                    if (aiControlledVehicles.find(vid) == aiControlledVehicles.end()) {
                        applyRuleBasedFallback(vid, mobilityByVehicle[vid], "ai_no_command_for_vehicle");
                    }
                }
            }
        }
        catch (const json::exception& e) {
            EV_WARN << "PythonCoSimManager: AI JSON parse error: " << e.what() << endl;
            if (ruleBasedFallbackEnabled) {
                for (const auto& vid : activeVehicles) {
                    applyRuleBasedFallback(vid, mobilityByVehicle[vid], "ai_parse_error");
                }
            }
        }
    }

    scheduleAt(simTime() + tickInterval, tickTimer);
}

void PythonCoSimManager::finish(cComponent* component, simsignal_t signalID)
{
    cListener::finish(component, signalID);
}

void PythonCoSimManager::finish()
{
    if (traciScenarioManager != nullptr) {
        traciScenarioManager->unsubscribe(veins::TraCIScenarioManager::traciTimestepEndSignal, this);
        traciScenarioManager->unsubscribe(veins::TraCIScenarioManager::traciTimestepSafetyEndSignal, this);
    }

    closeDestinationTrackingLog();

    if (udpSocket >= 0) {
        close(udpSocket);
        udpSocket = -1;
    }
    cancelAndDelete(tickTimer);
    tickTimer = nullptr;
}
