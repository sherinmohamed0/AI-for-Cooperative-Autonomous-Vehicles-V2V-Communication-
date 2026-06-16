//
// UDP co-simulation bridge to the external AI model (live_master_server.py).
// AI is primary; rule-based safety shield caps every AI command; full rule-based
// fallback on timeout / missing commands; post-inject shield on traciTimestepSafetyEnd.
//

package org.car2x.veins.modules.application.traci;

simple PythonCoSimManager {
    parameters:
        @display("p=50,50;i=block/network2");

        bool enabled = default(true);
        string aiServerHost = default("100.95.71.73");
        int aiServerPort = default(9999);
        
        double tickInterval @unit(s) = default(0.1s);
        // Max wait for AI UDP reply before rule-based fallback (ms). Should be < tickInterval.
        double aiResponseTimeoutMs @unit(ms) = default(5000ms);
        
        bool ruleBasedFallbackEnabled = default(true);
        // Cap / override AI commands using rule-based TTC, gap, and emergency lane change.
        bool safetyShieldEnabled = default(true);
        // Rule-based adjacent-lane escape when AI lateral output is too weak.
        bool proactiveLaneEscapeEnabled = default(true);
        double proactiveLaneEscapeLateralCommand = default(2.0);
        // Normal |a_x| threshold for AI lane change; lowered automatically when gap is small.
        double aiLaneChangeThreshold = default(1.5);
        double aiLaneChangeUrgentThreshold = default(0.3);
        double aiLaneChangeUrgentGapM @unit(m) = default(25m);
        
        // Unified log: every applied decision (AI or rule_based) with type braking/lane_change/etc.
        string decisionSummaryPath = default("driving_decisions_summary.txt");
        // Per-control-step vehicle destination progress log (comma-separated).
        string destinationTrackingPath = default("vehicle_destination_tracking.txt");
        // SUMO routes file used to resolve planned destinations (e.g. scenario1.rou.xml).
        string vehicleRouteFilePath = default("scenario1.rou.xml");
        int closeDestinationEdgeThreshold = default(2);
        double destinationTrackingInterval @unit(s) = default(1s);

        // Rule-based fallback parameters (same semantics as TraCIDemo11p)
        double ruleSafeTimeHeadway @unit(s) = default(1.3s);
        double ruleMinSpatialGap @unit(m) = default(8.0m);
        double ruleLeaderSearchDistance @unit(m) = default(200.0m);
        double ruleControlHorizon @unit(s) = default(0.2s);
        double ruleIdmAccel = default(2.0);
        double ruleIdmComfortDecel = default(3.0);
        double ruleKinematicDecel = default(6.5);
        double ruleMaxCmdDecel = default(9.0);
        double ruleTtcThreshold @unit(s) = default(4s);
        double ruleIdmDelta = default(4.0);
        double ruleEmergencyMinClosingSpeed = default(0.3);
        double ruleEmergencyTtcThreshold @unit(s) = default(5s);
        double ruleEmergencyGapThreshold @unit(m) = default(28m);
        double ruleBrakingSufficientMargin = default(1.0);
        double ruleProactiveLaneChangeGapM @unit(m) = default(28m);
        double ruleProactiveStoppedLeaderMaxSpeedMps = default(0.5); // unitless rate m/s
        double ruleLaneChangeTraCIDuration @unit(s) = default(4s);
        int ruleEmergencyLaneChangeMode = default(1398101);
        int ruleDefaultLaneChangeMode = default(31);
        double ruleJunctionNearbyRadiusM @unit(m) = default(40m);
        double ruleJunctionMinSafeGapM @unit(m) = default(5.0m);
}