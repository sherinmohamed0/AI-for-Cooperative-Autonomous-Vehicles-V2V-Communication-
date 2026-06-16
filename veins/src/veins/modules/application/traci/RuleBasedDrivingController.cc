//
// SPDX-License-Identifier: GPL-2.0-or-later
//

#include "veins/modules/application/traci/RuleBasedDrivingController.h"

#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"
#include "veins/veins.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <list>
#include <sstream>

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

namespace veins {

namespace {

using TraCIConstants::LANECHANGE_LEFT;
using TraCIConstants::LANECHANGE_RIGHT;

const double MPS2_TO_FTPS2 = 3.28083989501312;

/** SUMO neighbor modes: see TraCI / Vehicle Value Retrieval / neighboring vehicles (0xbf). */
const uint8_t NB_LEFT_AHEAD_BLOCKING = 6; // left, ahead, blocking only
const uint8_t NB_LEFT_BEHIND_BLOCKING = 4;
const uint8_t NB_RIGHT_AHEAD_BLOCKING = 7;
const uint8_t NB_RIGHT_BEHIND_BLOCKING = 5;

double clampd(double x, double lo, double hi)
{
    return std::min(hi, std::max(lo, x));
}

double wrapPi(double a)
{
    const double twoPi = 2.0 * M_PI;
    while (a > M_PI)
        a -= twoPi;
    while (a < -M_PI)
        a += twoPi;
    return a;
}

double closestPointOnSegment(const Coord& p, const Coord& a, const Coord& b, Coord& closest)
{
    Coord ab = b - a;
    double ab2 = ab.x * ab.x + ab.y * ab.y;
    if (ab2 < 1e-18) {
        closest = a;
        Coord d = p - a;
        return d.x * d.x + d.y * d.y;
    }
    Coord ap = p - a;
    double t = (ap.x * ab.x + ap.y * ab.y) / ab2;
    t = clampd(t, 0.0, 1.0);
    closest = Coord(a.x + ab.x * t, a.y + ab.y * t, 0.0);
    Coord d = p - closest;
    return d.x * d.x + d.y * d.y;
}

double computeLaneHeadingForLaneId(TraCICommandInterface& ci, TraCIMobility* tm, const std::string& laneId)
{
    if (!tm || laneId.empty())
        return std::numeric_limits<double>::quiet_NaN();

    std::list<Coord> shape = ci.lane(laneId).getShape();
    if (shape.size() < 2)
        return std::numeric_limits<double>::quiet_NaN();

    Coord p = tm->getPositionAt(simTime());
    auto it = shape.begin();
    Coord prev = *it;
    ++it;
    double bestD2 = std::numeric_limits<double>::infinity();
    Coord bestA = prev;
    Coord bestB = prev;
    for (; it != shape.end(); ++it) {
        Coord a = prev;
        Coord b = *it;
        Coord closest;
        double d2 = closestPointOnSegment(p, a, b, closest);
        if (d2 < bestD2) {
            bestD2 = d2;
            bestA = a;
            bestB = b;
        }
        prev = *it;
    }

    Coord seg = bestB - bestA;
    if (seg.x * seg.x + seg.y * seg.y < 1e-18)
        return std::numeric_limits<double>::quiet_NaN();

    return std::atan2(seg.y, seg.x);
}

double computeLaneHeadingRad(TraCICommandInterface& ci, TraCIMobility* tm, TraCICommandInterface::Vehicle& ego)
{
    return computeLaneHeadingForLaneId(ci, tm, ego.getLaneId());
}

std::string makeLaneIdFromEdgeIndex(const std::string& edgeId, int laneIndex)
{
    return edgeId + "_" + std::to_string(laneIndex);
}

bool laneChangeAllowedByPermissions(TraCICommandInterface& ci, const std::string& currentLaneId, int8_t direction)
{
    std::list<std::string> perms = ci.lane(currentLaneId).getChangePermissions(direction);
    return perms.empty();
}

bool neighborsClearForLeft(TraCICommandInterface::Vehicle& ego)
{
    try {
        return ego.getNeighborIds(NB_LEFT_AHEAD_BLOCKING).empty() && ego.getNeighborIds(NB_LEFT_BEHIND_BLOCKING).empty();
    }
    catch (const cException&) {
        return false;
    }
    catch (const std::exception&) {
        return false;
    }
}

bool neighborsClearForRight(TraCICommandInterface::Vehicle& ego)
{
    try {
        return ego.getNeighborIds(NB_RIGHT_AHEAD_BLOCKING).empty() && ego.getNeighborIds(NB_RIGHT_BEHIND_BLOCKING).empty();
    }
    catch (const cException&) {
        return false;
    }
    catch (const std::exception&) {
        return false;
    }
}

bool tryAdjacentLaneEscape(
    TraCICommandInterface& ci,
    TraCICommandInterface::Vehicle& ego,
    const RuleBasedDrivingController::Params& p,
    int& targetLaneIndex,
    std::string& targetLaneId,
    int32_t& appliedLaneChangeMode,
    double& laneChangeDuration)
{
    targetLaneIndex = -1;
    targetLaneId.clear();
    appliedLaneChangeMode = p.emergencyLaneChangeMode;
    laneChangeDuration = std::max(0.5, p.laneChangeTraCIDuration);

    try {
        const std::string edgeId = ci.lane(ego.getLaneId()).getRoadId();
        const int nLanes = static_cast<int>(ci.road(edgeId).getLaneNumber());
        const int idx = static_cast<int>(ego.getLaneIndex());

        const bool tryLeft = (idx + 1 < nLanes) && laneChangeAllowedByPermissions(ci, ego.getLaneId(), LANECHANGE_LEFT) && neighborsClearForLeft(ego);
        const bool tryRight = (idx - 1 >= 0) && laneChangeAllowedByPermissions(ci, ego.getLaneId(), LANECHANGE_RIGHT) && neighborsClearForRight(ego);

        if (tryLeft) {
            targetLaneIndex = idx + 1;
            targetLaneId = makeLaneIdFromEdgeIndex(edgeId, targetLaneIndex);
            return true;
        }
        if (tryRight) {
            targetLaneIndex = idx - 1;
            targetLaneId = makeLaneIdFromEdgeIndex(edgeId, targetLaneIndex);
            return true;
        }
    }
    catch (...) {
    }

    return false;
}

bool brakingDistanceSufficient(double vEgo, double vLead, double gap, double maxDecel, double minGap, double margin)
{
    if (gap <= minGap)
        return false;
    const double dv = vEgo - vLead;
    if (dv <= 0.05)
        return true;
    const double usable = std::max(0.0, gap - minGap);
    const double distNeed = (dv * dv) / std::max(1e-3, 2.0 * maxDecel);
    return usable >= margin * distNeed;
}

struct SpeedAdvice {
    double vTarget = 0.0;
    bool mitigated = false;
    std::string leaderId;
    double gap = std::numeric_limits<double>::quiet_NaN();
    /** True when longitudinal control uses 2D Euclidean proximity (junction lanes). */
    bool junction2DMode = false;
};

/** SUMO internal/junction lanes contain ':'; normal edges do not. */
bool isJunctionLane(const std::string& laneId)
{
    return laneId.find(':') != std::string::npos;
}

struct ClosestVehicle2D {
    std::string id;
    double distM = std::numeric_limits<double>::infinity();
};

/**
 * Junction 2D mode: closest other vehicle within searchRadiusM (air distance).
 * Skips egoId; ignores vehicles that leave the simulation mid-query.
 */
ClosestVehicle2D findClosestVehicle2D(
    TraCICommandInterface& ci,
    TraCIMobility* tm,
    const std::string& egoId,
    double searchRadiusM)
{
    ClosestVehicle2D out;
    if (!tm || egoId.empty() || searchRadiusM <= 0.0) {
        return out;
    }

    const Coord egoPos = tm->getPositionAt(simTime());
    const double r2 = searchRadiusM * searchRadiusM;

    std::list<std::string> ids;
    try {
        ids = ci.getVehicleIds();
    }
    catch (const std::exception&) {
        return out;
    }
    catch (...) {
        return out;
    }

    for (const auto& vid : ids) {
        if (vid == egoId) {
            continue;
        }
        try {
            TraCICommandInterface::Vehicle other = ci.vehicle(vid);
            const Coord op = other.getPosition();
            const double dx = op.x - egoPos.x;
            const double dy = op.y - egoPos.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 > r2) {
                continue;
            }
            const double d = std::sqrt(d2);
            if (d < out.distM) {
                out.distM = d;
                out.id = vid;
            }
        }
        catch (const std::exception&) {
            continue;
        }
        catch (...) {
            continue;
        }
    }
    return out;
}

/** Shared IDM / TTC cap given a leader id and longitudinal or 2D gap (m). */
SpeedAdvice fillSpeedAdviceFromGap(
    TraCICommandInterface& ci,
    TraCICommandInterface::Vehicle& ego,
    const RuleBasedDrivingController::Params& p,
    const std::string& leaderId,
    double gap,
    bool junction2DMode)
{
    SpeedAdvice adv;
    adv.junction2DMode = junction2DMode;
    const double v0 = std::max(0.1, ego.getMaxSpeed());
    const double v = std::max(0.0, ego.getSpeed());
    const double dt = clampd(p.controlHorizon, 1e-3, 1.0);
    const double s0 = std::max(0.5, p.minSpatialGap);
    const double T = std::max(0.2, p.safeTimeHeadway);
    const double a = std::max(0.1, std::min(p.idmA, ego.getAccel()));
    const double b = std::max(0.1, p.idmB);
    const double bKin = std::max(0.1, p.kinematicDecel);
    const double dMax = std::max(a, p.maxCmdDecel);
    const double delta = std::max(1.0, p.idmDelta);

    adv.leaderId = leaderId;
    adv.gap = gap;

    if (!std::isfinite(gap) || gap <= 0.0) {
        adv.vTarget = 0.0;
        adv.mitigated = true;
        return adv;
    }

    double vLead = 0.0;
    try {
        TraCICommandInterface::Vehicle lead = ci.vehicle(leaderId);
        vLead = std::max(0.0, lead.getSpeed());
    }
    catch (const std::exception&) {
        adv.vTarget = std::max(0.0, v - dMax * dt);
        adv.mitigated = true;
        return adv;
    }
    catch (...) {
        adv.vTarget = std::max(0.0, v - dMax * dt);
        adv.mitigated = true;
        return adv;
    }

    const double dv = v - vLead;

    bool haveKinCap = false;
    double vKinCap = v0;
    if (gap > s0) {
        haveKinCap = true;
        vKinCap = vLead + std::sqrt(std::max(0.0, 2.0 * bKin * (gap - s0)));
    }
    else {
        haveKinCap = true;
        vKinCap = vLead;
    }

    bool haveTtcCap = false;
    double vTtcCap = v0;
    if (dv > 0.05 && p.ttcThreshold > 0.0) {
        const double ttc = gap / dv;
        if (ttc < p.ttcThreshold) {
            haveTtcCap = true;
            const double urgency = clampd((p.ttcThreshold - ttc) / p.ttcThreshold, 0.0, 1.0);
            const double decel = dMax * (0.5 + 0.5 * urgency);
            vTtcCap = std::max(0.0, v - decel * dt);
            vTtcCap = std::min(vTtcCap, vLead + gap / std::max(T, 0.3));
        }
    }

    double sStar = s0 + v * T;
    if (a > 1e-6 && b > 1e-6) {
        sStar += v * dv / (2.0 * std::sqrt(a * b));
    }
    sStar = std::max(s0, sStar);

    const double aFree = a * (1.0 - std::pow(v / v0, delta));
    const double aInt = -a * std::pow(sStar / std::max(gap, 0.05), 2.0);
    double aCmd = aFree + aInt;
    aCmd = clampd(aCmd, -dMax, a);

    double vIdm = v + aCmd * dt;
    vIdm = clampd(vIdm, 0.0, v0);

    double vTarget = vIdm;
    vTarget = std::min(vTarget, v0);
    if (haveKinCap)
        vTarget = std::min(vTarget, vKinCap);
    if (haveTtcCap)
        vTarget = std::min(vTarget, vTtcCap);

    const double criticalGap = junction2DMode ? std::max(s0, p.junctionMinSafeGapM) : s0;
    if (gap <= criticalGap) {
        vTarget = std::min(vTarget, v * p.criticalGapScale);
        vTarget = std::min(vTarget, vLead);
        vTarget = std::min(vTarget, vKinCap);
    }

    const double margin = std::max(0.15, s0 * 0.25);
    const double usableGap = std::max(0.0, gap - margin);
    const double vStopFromGap = std::sqrt(std::max(0.0, 2.0 * dMax * usableGap));
    const double vEnergyCap = vLead + vStopFromGap;
    vTarget = std::min(vTarget, vEnergyCap);

    adv.vTarget = std::max(0.0, vTarget);

    const double hazardWindow = std::max(3.0 * s0, 12.0);
    const bool inHazardBand = gap < hazardWindow || gap <= criticalGap;
    adv.mitigated = inHazardBand && (v - adv.vTarget > 0.2);
    if (gap <= criticalGap && v > 0.05) {
        adv.mitigated = true;
    }

    return adv;
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

std::string resolveOutputPath(const std::string& configuredPath)
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

void appendAvoidanceSummaryLine(
    const RuleBasedDrivingController::Params& p,
    double simTimeS,
    const std::string& leaderId,
    double gap,
    double vEgo,
    double vCmd,
    const char* mitigationTag,
    const char* safetyAction,
    const std::string& targetLane)
{
    if (p.avoidanceSummaryPath.empty()) {
        return;
    }

    const std::string path = resolveOutputPath(p.avoidanceSummaryPath);
    bool writeHeader = false;
    {
        std::ifstream tin(path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
        if (!tin.is_open() || tin.tellg() == 0) {
            writeHeader = true;
        }
    }

    std::ofstream out(path.c_str(), std::ios::out | std::ios::app);
    if (!out.is_open()) {
        return;
    }

    if (writeHeader) {
        out << "# RuleBasedDrivingController: mitigations vs rear-end hazard (e.g. TraCI collision inject).\n";
        out << "# sim_time_s\tego_id\tleader_id\tgap_m\tego_speed_ms\tcommanded_speed_ms\tmitigation_tag\tsafety_action\ttarget_lane\n";
    }

    const char* laneOut = targetLane.empty() ? "none" : targetLane.c_str();
    out << std::fixed << std::setprecision(6) << simTimeS << "\t" << p.logVehicleId << "\t" << leaderId << "\t" << gap << "\t" << vEgo << "\t" << vCmd << "\t" << mitigationTag << "\t" << safetyAction << "\t" << laneOut << "\n";
    out.flush();
}

SpeedAdvice computeSpeedAdvice(
    TraCICommandInterface& ci,
    TraCIMobility* tm,
    TraCICommandInterface::Vehicle& ego,
    const RuleBasedDrivingController::Params& p)
{
    SpeedAdvice adv;
    const double v0 = std::max(0.1, ego.getMaxSpeed());
    const double v = std::max(0.0, ego.getSpeed());
    const double dt = clampd(p.controlHorizon, 1e-3, 1.0);
    const double a = std::max(0.1, std::min(p.idmA, ego.getAccel()));
    const double delta = std::max(1.0, p.idmDelta);

    std::string laneId;
    try {
        laneId = ego.getLaneId();
    }
    catch (const std::exception&) {
        laneId.clear();
    }
    catch (...) {
        laneId.clear();
    }

    const std::string egoId = !p.logVehicleId.empty() ? p.logVehicleId : ego.getId();

    // Junction lanes (':' in id): 2D Euclidean proximity — bypass getLeader() phantom overlaps.
    if (isJunctionLane(laneId)) {
        const double searchR = std::max(1.0, p.junctionNearbyRadiusM);
        const ClosestVehicle2D closest = findClosestVehicle2D(ci, tm, egoId, searchR);
        if (closest.id.empty() || !std::isfinite(closest.distM)) {
            const double aFree = a * (1.0 - std::pow(v / v0, delta));
            double vNext = v + aFree * dt;
            adv.vTarget = clampd(vNext, 0.0, v0);
            adv.mitigated = false;
            adv.junction2DMode = true;
            return adv;
        }
        return fillSpeedAdviceFromGap(ci, ego, p, closest.id, closest.distM, true);
    }

    // Normal road: 1D getLeader() longitudinal gap.
    auto leaderInfo = ego.getLeader(p.leaderSearchDistance);
    if (leaderInfo.first.empty()) {
        const double aFree = a * (1.0 - std::pow(v / v0, delta));
        double vNext = v + aFree * dt;
        adv.vTarget = clampd(vNext, 0.0, v0);
        adv.mitigated = false;
        return adv;
    }

    return fillSpeedAdviceFromGap(ci, ego, p, leaderInfo.first, leaderInfo.second, false);
}

} // namespace

const char* RuleBasedDrivingController::safetyActionLabel(SafetyAction a)
{
    switch (a) {
    case SafetyAction::None:
        return "none";
    case SafetyAction::LaneFollow:
        return "lane_follow";
    case SafetyAction::EmergencyBraking:
        return "emergency_brake";
    case SafetyAction::EmergencyLaneChange:
        return "emergency_lane_change";
    }
    return "unknown";
}

void RuleBasedDrivingController::appendSafetyDecisionLog(const Params& p, double simTimeS, const Command& cmd, const std::string& collisionOutcome)
{
    if (p.safetyDecisionLogPath.empty()) {
        return;
    }
    const std::string path = resolveOutputPath(p.safetyDecisionLogPath);
    bool writeHeader = false;
    {
        std::ifstream tin(path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
        if (!tin.is_open() || tin.tellg() == 0) {
            writeHeader = true;
        }
    }
    std::ofstream out(path.c_str(), std::ios::out | std::ios::app);
    if (!out.is_open()) {
        return;
    }
    if (writeHeader) {
        out << "# RuleBasedDrivingController detailed safety log (tab-separated).\n";
        out << "# sim_time_s\tvehicle_id\tdanger_description\tttc_s\trelative_speed_m_s\tgap_m\t";
        out << "safety_action\tbraking_alone_sufficient\taccel_command_m_s2\tsteer_command_rad\t";
        out << "target_lane_id\ttarget_lane_index\ttra_lane_change_requested\tcollision_avoidance_outcome\n";
    }
    out << std::fixed << std::setprecision(6) << simTimeS << "\t" << p.logVehicleId << "\t" << cmd.dangerDescription << "\t";
    out << cmd.ttcSec << "\t" << cmd.relativeSpeedMps << "\t" << cmd.gapM << "\t";
    out << safetyActionLabel(cmd.safetyAction) << "\t" << (cmd.brakingAloneSufficient ? "yes" : "no") << "\t";
    out << cmd.accelMps2 << "\t" << cmd.steerRad << "\t" << cmd.targetLaneId << "\t" << cmd.targetLaneIndex << "\t";
    out << (cmd.requestTraCILaneChange ? "yes" : "no") << "\t" << collisionOutcome << "\n";
    out.flush();
}

RuleBasedDrivingController::Command RuleBasedDrivingController::computeCommand(
    TraCICommandInterface& ci,
    TraCIMobility* tm,
    TraCICommandInterface::Vehicle& ego,
    const Params& p,
    double dtSec)
{
    Command out;
    const double v0 = std::max(0.1, ego.getMaxSpeed());
    const double v = std::max(0.0, ego.getSpeed());
    double dt = dtSec;
    if (!std::isfinite(dt) || dt < 1e-4)
        dt = clampd(p.controlHorizon, 1e-3, 1.0);

    SpeedAdvice advice = computeSpeedAdvice(ci, tm, ego, p);
    double vTarget = advice.vTarget;

    const double gap = advice.gap;
    double vLead = 0.0;
    double dv = 0.0;
    double ttc = std::numeric_limits<double>::infinity();

    if (!advice.leaderId.empty() && std::isfinite(gap)) {
        out.leaderId = advice.leaderId;
        out.gapM = gap;
        try {
            vLead = std::max(0.0, ci.vehicle(advice.leaderId).getSpeed());
        }
        catch (...) {
            vLead = 0.0;
        }
        dv = v - vLead;
        out.relativeSpeedMps = dv;
        if (dv > p.emergencyMinClosingSpeed) {
            ttc = gap / dv;
            out.ttcSec = ttc;
        }
        else {
            out.ttcSec = std::numeric_limits<double>::infinity();
        }
    }

    const bool closing = dv > p.emergencyMinClosingSpeed;
    const bool ttcHazard = std::isfinite(ttc) && ttc < p.emergencyTtcThreshold;
    const bool gapHazard = std::isfinite(gap) && gap < p.emergencyGapThreshold;
    const bool junctionHardStop = advice.junction2DMode && std::isfinite(gap) && gap < std::max(p.minSpatialGap, p.junctionMinSafeGapM);
    const bool emergency = !advice.leaderId.empty()
        && (junctionHardStop || (closing && (ttcHazard || gapHazard)));

    std::string steerLaneId = ego.getLaneId();
    out.safetyAction = SafetyAction::LaneFollow;

    const double s0 = std::max(0.5, p.minSpatialGap);

    // Proactive lane escape behind a stopped/slow leader (efficiency — avoid unnecessary full stops).
    if (p.proactiveLaneChangeGapM > 0.0
        && !advice.leaderId.empty()
        && std::isfinite(gap)
        && vLead <= p.proactiveStoppedLeaderMaxSpeedMps
        && gap > s0
        && gap < p.proactiveLaneChangeGapM
        && v > 0.15
        && closing) {
        int escapeLaneIndex = -1;
        std::string escapeLaneId;
        int32_t escapeLaneMode = 0;
        double escapeDuration = 0.0;
        if (tryAdjacentLaneEscape(ci, ego, p, escapeLaneIndex, escapeLaneId, escapeLaneMode, escapeDuration)) {
            out.targetLaneIndex = escapeLaneIndex;
            out.targetLaneId = escapeLaneId;
            out.requestTraCILaneChange = true;
            out.appliedLaneChangeMode = escapeLaneMode;
            out.laneChangeDuration = escapeDuration;
            steerLaneId = out.targetLaneId;
            out.safetyAction = SafetyAction::EmergencyLaneChange;
            out.dangerDescription = "proactive_lane_escape leader=" + advice.leaderId + " gap=" + std::to_string(gap) + "m";
            out.gapM = gap;
            out.relativeSpeedMps = dv;
            if (std::isfinite(ttc)) {
                out.ttcSec = ttc;
            }
        }
    }

    if (emergency) {
        std::ostringstream ddesc;
        if (advice.junction2DMode) {
            ddesc << "junction_2d_proximity threat=" << advice.leaderId;
        }
        else {
            ddesc << "closing_on_leader leader=" << advice.leaderId;
        }
        if (ttcHazard)
            ddesc << " ttc=" << ttc << "s";
        if (gapHazard || junctionHardStop)
            ddesc << " gap=" << gap << "m";
        out.dangerDescription = ddesc.str();

        const double dMax = std::max(p.idmA, p.maxCmdDecel);
        out.brakingAloneSufficient = brakingDistanceSufficient(v, vLead, gap, dMax, s0, p.brakingSufficientMargin);

        bool choseLaneChange = out.requestTraCILaneChange;
        if (!choseLaneChange && !out.brakingAloneSufficient) {
            int escapeLaneIndex = -1;
            std::string escapeLaneId;
            int32_t escapeLaneMode = 0;
            double escapeDuration = 0.0;
            if (tryAdjacentLaneEscape(ci, ego, p, escapeLaneIndex, escapeLaneId, escapeLaneMode, escapeDuration)) {
                out.targetLaneIndex = escapeLaneIndex;
                out.targetLaneId = escapeLaneId;
                out.requestTraCILaneChange = true;
                out.appliedLaneChangeMode = escapeLaneMode;
                out.laneChangeDuration = escapeDuration;
                steerLaneId = out.targetLaneId;
                out.safetyAction = SafetyAction::EmergencyLaneChange;
                choseLaneChange = true;
            }
        }

        if (!choseLaneChange) {
            out.safetyAction = SafetyAction::EmergencyBraking;
            out.requestTraCILaneChange = false;
            out.targetLaneIndex = -1;
            out.targetLaneId.clear();
        }
    }
    else {
        out.dangerDescription.clear();
        out.ttcSec = std::numeric_limits<double>::quiet_NaN();
        out.relativeSpeedMps = std::numeric_limits<double>::quiet_NaN();
        out.gapM = std::numeric_limits<double>::quiet_NaN();
        out.brakingAloneSufficient = false;
    }

    double rawAccel = (vTarget - v) / dt;
    rawAccel = clampd(rawAccel, -p.maxCmdDecel, p.idmA);
    double vNext = v + rawAccel * dt;
    vNext = clampd(vNext, 0.0, v0);
    out.accelMps2 = (vNext - v) / dt;
    out.appliedSpeedMps = vNext;

    if (advice.mitigated && !p.avoidanceSummaryPath.empty()) {
        const char* tag = advice.junction2DMode ? "junction_2d_cap"
            : ((std::isfinite(advice.gap) && advice.gap <= 0.0) ? "overlap_stop" : "rear_end_rule_cap");
        appendAvoidanceSummaryLine(
            p,
            SIMTIME_DBL(simTime()),
            advice.leaderId,
            advice.gap,
            v,
            out.appliedSpeedMps,
            tag,
            safetyActionLabel(out.safetyAction),
            out.targetLaneId);
    }

    const double laneH = computeLaneHeadingForLaneId(ci, tm, steerLaneId);
    if (std::isfinite(laneH) && tm) {
        const double hostH = tm->getHeading().getRad();
        const double err = wrapPi(laneH - hostH);
        out.steerRad = clampd(p.steerLaneGain * err, -p.steerMaxRad, p.steerMaxRad);
    }
    else {
        const double fallback = computeLaneHeadingRad(ci, tm, ego);
        if (std::isfinite(fallback) && tm) {
            const double hostH = tm->getHeading().getRad();
            const double err = wrapPi(fallback - hostH);
            out.steerRad = clampd(p.steerLaneGain * err, -p.steerMaxRad, p.steerMaxRad);
        }
    }

    return out;
}

std::string RuleBasedDrivingController::formatAccelFtSteerRad(const Command& c)
{
    const double aFt = c.accelMps2 * MPS2_TO_FTPS2;
    std::ostringstream os;
    os << "Accel: " << std::fixed << std::setprecision(3) << aFt << " ft/s² | Steer: " << std::setprecision(3) << c.steerRad << " rad | " << safetyActionLabel(c.safetyAction);
    return os.str();
}

} // namespace veins
