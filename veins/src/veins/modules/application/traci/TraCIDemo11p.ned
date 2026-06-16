package org.car2x.veins.modules.application.traci;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple TraCIDemo11p extends DemoBaseApplLayer
{
    parameters:
        @class(veins::TraCIDemo11p);
        @display("i=block/app2");

        // --- Rule-based longitudinal control (no sockets / no AI) ---
        bool ruleBasedControlEnabled = default(false);
        double ruleSafeTimeHeadway @unit(s) = default(1.5s);
        double ruleMinSpatialGap @unit(m) = default(4.0m);
        double ruleLeaderSearchDistance @unit(m) = default(200.0m);
        double ruleControlHorizon @unit(s) = default(0.1s);
        double ruleIdmAccel = default(2.0);
        double ruleIdmComfortDecel = default(3.0);
        double ruleKinematicDecel = default(4.5);
        double ruleMaxCmdDecel = default(6.0);
        double ruleTtcThreshold @unit(s) = default(2s);
        double ruleIdmDelta = default(4.0);
        double ruleSteerLaneGain = default(0.85);
        double ruleSteerMaxRad = default(1.2);
        
        // TSV log (append + flush) when rule-based braking mitigates a rear-end hazard
        string ruleAvoidanceSummaryPath = default("rule_based_collision_avoidance_summary.txt");
        // Detailed safety log (TTC, action, steering, lane, outcome).
        string ruleSafetyDecisionLogPath = default("rule_based_safety_decisions.txt");
        bool ruleSumoSafetyColorEnabled = default(true);
        
        double ruleEmergencyMinClosingSpeed = default(0.3);
        double ruleEmergencyTtcThreshold @unit(s) = default(5s);
        double ruleEmergencyGapThreshold @unit(m) = default(40m);
        double ruleBrakingSufficientMargin = default(1.2);
        double ruleLaneChangeTraCIDuration @unit(s) = default(4s);
        int ruleEmergencyLaneChangeMode = default(1398101);
        int ruleDefaultLaneChangeMode = default(31);
        
        // Missing parameter added so it doesn't crash when reading omnetpp.ini
        double junctionNearbyRadiusM @unit(m) = default(40m);

        // Signals for OMNeT++ logging
        @signal[sigVehicleId](type="long");
        @signal[sigFrameId](type="long");       
        @signal[sigTotalFrames](type="long");
        @signal[sigGlobalTime](type="double");
        @signal[sigLocalX](type="double");
        @signal[sigLocalY](type="double");
        @signal[sigGlobalX](type="double");
        @signal[sigGlobalY](type="double");
        @signal[sigVel](type="double");
        @signal[sigAcc](type="double");
        @signal[sigPreceding](type="long");
        @signal[sigFollowing](type="long");
        @signal[sigSpaceHeadway](type="double");
        @signal[sigTimeHeadway](type="double");
        
        // Signals for recording data to the results/CSV
        @signal[sigVLength](type="double");
        @signal[sigVWidth](type="double");
        @signal[sigVClass](type="long");
        @signal[sigLaneIdNumeric](type="long");
        
        // Statistics definitions
        @statistic[sigVehicleId](title="Vehicle ID"; record=vector);
        @statistic[sigFrameId](title="Frame ID"; record=vector);       
        @statistic[sigTotalFrames](title="Total Frames"; record=vector);
        @statistic[sigGlobalTime](title="Global Time"; record=vector);
        @statistic[sigLocalX](title="Local X"; record=vector);
        @statistic[sigLocalY](title="Local Y"; record=vector);
        @statistic[sigGlobalX](title="Global X"; record=vector);
        @statistic[sigGlobalY](title="Global Y"; record=vector);
        @statistic[sigVel](title="Velocity"; record=vector);
        @statistic[sigAcc](title="Acceleration"; record=vector);
        @statistic[sigPreceding](title="Preceding ID"; record=vector);
        @statistic[sigFollowing](title="Following ID"; record=vector);
        @statistic[sigSpaceHeadway](title="Space Headway"; record=vector);
        @statistic[sigTimeHeadway](title="Time Headway"; record=vector);
        
        // Statistics to tell OMNeT++ to create a data vector for each signal
        @statistic[sigVLength](title="Vehicle Length"; source="sigVLength"; record=vector);
        @statistic[sigVWidth](title="Vehicle Width"; source="sigVWidth"; record=vector);
        @statistic[sigVClass](title="Vehicle Class"; source="sigVClass"; record=vector);
        @statistic[sigLaneIdNumeric](title="Numeric Lane ID"; record=vector);
}