//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Rule-based longitudinal control + lane-heading / emergency escape steering (no sockets, no ML).
// Longitudinal output is applied via TraCI setSpeed from the application layer (after
// TraCISumoCollisionInjector on traciTimestepEndSignal). Host applications should subscribe to
// TraCIScenarioManager::traciTimestepSafetyEndSignal so setSpeed runs after inject. Optional emergency
// lane change uses TraCI changeLane from the application after computeCommand.
//

#pragma once

#include <limits>
#include <string>

#include "veins/modules/mobility/traci/TraCICommandInterface.h"

namespace veins {

class TraCIMobility;

class VEINS_API RuleBasedDrivingController {
public:
    enum class SafetyAction {
        None,
        LaneFollow,
        EmergencyBraking,
        EmergencyLaneChange
    };

    struct VEINS_API Params {
        double safeTimeHeadway = 1.5;
        double minSpatialGap = 4.0;
        double leaderSearchDistance = 200.0;
        /** Fallback horizon (s) when position-update dt is tiny. */
        double controlHorizon = 0.05;
        double idmA = 2.0;
        double idmB = 3.0;
        double kinematicDecel = 4.5;
        double maxCmdDecel = 6.0;
        double ttcThreshold = 2.0;
        double criticalGapScale = 0.35;
        double idmDelta = 4.0;
        /** Lane heading error gain (maps rad error to reported steer command). */
        double steerLaneGain = 0.85;
        /** Cap on reported steer magnitude (rad). */
        double steerMaxRad = 1.2;
        /** If non-empty, append TSV lines when a hazardous closing situation is mitigated (see .cc). */
        std::string avoidanceSummaryPath;
        /** Detailed safety decision log (sim time, TTC, action, steer, lane, outcome hint). */
        std::string safetyDecisionLogPath;
        /** Caller sets each step (SUMO vehicle id) before computeCommand when logging is enabled. */
        std::string logVehicleId;

        /** Engage emergency decision logic when leader present and closing speed exceeds this (m/s). */
        double emergencyMinClosingSpeed = 0.3;
        /** Engage when forward time-to-collision falls below this (s); inf bits ignored when dv<=0. */
        double emergencyTtcThreshold = 5.0;
        /** Also engage emergency logic when gap (m) is below this and closing. */
        double emergencyGapThreshold = 40.0;
        /** If true, prefer braking when this margin times required braking distance still fits in gap. */
        double brakingSufficientMargin = 1.2;
        /** SUMO TraCI lane-change keep duration (s). */
        double laneChangeTraCIDuration = 4.0;
        /** Lane-change mode bitmask while executing emergency lateral escape (SUMO default is type-dependent). */
        int32_t emergencyLaneChangeMode = 0x155555;
        /** Default SUMO lane-change mode to restore after emergency (typical default 31). */
        int32_t defaultLaneChangeMode = 31;
        /** Junction 2D mode: TraCI search radius for nearby vehicles (m). */
        double junctionNearbyRadiusM = 30.0;
        /** Junction 2D mode: minimum Euclidean separation before hard safety stop (m). */
        double junctionMinSafeGapM = 5.0;
        /**
         * When leader speed is below @ref proactiveStoppedLeaderMaxSpeedMps and gap is within this
         * range, attempt adjacent-lane escape before coming to a full stop (0 disables).
         */
        double proactiveLaneChangeGapM = 28.0;
        /** Leader considered "stopped" for proactive lane escape below this speed (m/s). */
        double proactiveStoppedLeaderMaxSpeedMps = 0.5;
    };

    struct VEINS_API Command {
        double accelMps2 = 0.0;
        /** Steering signal toward current lane centerline or toward escape lane heading (rad). */
        double steerRad = 0.0;
        /** Speed applied via TraCI setSpeed after accel integration (m/s). */
        double appliedSpeedMps = 0.0;

        SafetyAction safetyAction = SafetyAction::None;
        std::string dangerDescription;
        double ttcSec = std::numeric_limits<double>::quiet_NaN();
        double relativeSpeedMps = std::numeric_limits<double>::quiet_NaN();
        double gapM = std::numeric_limits<double>::quiet_NaN();
        std::string leaderId;
        /** True when kinematic braking-distance check says braking alone is enough. */
        bool brakingAloneSufficient = false;
        /** Target absolute lane index for TraCI changeLane (SUMO: 0 = rightmost); -1 if none. */
        int targetLaneIndex = -1;
        std::string targetLaneId;
        /** Application should call TraCI changeLane when true. */
        bool requestTraCILaneChange = false;
        int32_t appliedLaneChangeMode = 0;
        double laneChangeDuration = 0.0;
    };

    static Command computeCommand(TraCICommandInterface& ci, TraCIMobility* tm, TraCICommandInterface::Vehicle& ego, const Params& p, double dtSec);

    static std::string formatAccelFtSteerRad(const Command& c);

    /** Human-readable safety action label for logs. */
    static const char* safetyActionLabel(SafetyAction a);

    /**
     * Append one TSV line to safetyDecisionLogPath (if configured).
     * @param collisionOutcome  e.g. pending / yes / no / critical_gap / -
     */
    static void appendSafetyDecisionLog(
        const Params& p,
        double simTimeS,
        const Command& cmd,
        const std::string& collisionOutcome);
};

} // namespace veins
