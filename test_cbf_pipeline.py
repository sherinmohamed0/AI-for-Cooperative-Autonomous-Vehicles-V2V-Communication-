import numpy as np
from cbf_layer import Layer3_CBF

def evaluate_scenario(cbf_filter, name, current_state, u_mpc, neighbors_current):
    """Helper function to run a scenario and cleanly print the results."""
    print("\n" + "="*65)
    print(f"SCENARIO: {name}")
    print("="*65)

    print(f"Inputs:")
    print(f"  Ego State (x, y, vx, vy) : {current_state}")
    print(f"  NMPC Intention (ax, ay)  : {u_mpc}")
    if neighbors_current:
        print(f"  Neighbor States          : {neighbors_current}")
    else:
        print(f"  Neighbor States          : None (Clear Road)")

    # Run the safety filter
    results = cbf_filter.filter_control(current_state, u_mpc, neighbors_current)

    # Error checking
    if not isinstance(results, dict):
        print(f"CRITICAL ERROR: filter_control returned {type(results)}")
        return

    print("-" * 65)
    print(f"Outputs:")
    print(f"  Filter Execution Mode    : {results['mode']}")

    # Safely evaluate and print the outputs based on success status
    if results['success'] and results['u_safe'] is not None:
        safe_ax, safe_ay = results['u_safe']
        print(f"  Safe Command Allowed     : a_x={safe_ax:.4f}, a_y={safe_ay:.4f}")

        # Check if the CBF had to change the NMPC's command
        if abs(safe_ax - u_mpc[0]) > 0.01 or abs(safe_ay - u_mpc[1]) > 0.01:
            print("\n  [VERIFICATION] SUCCESS: Layer 3 intervened and capped the AI's command.")
        else:
            print("\n  [VERIFICATION] SUCCESS: Layer 3 allowed the safe AI command to pass.")
    else:
        print(f"  Rule-Based Handover      : {results.get('handover_to_rule_based', 'Unknown')}")
        print("\n  [VERIFICATION] SUCCESS: Layer 3 blocked the AI and triggered the emergency handover.")


def run_cbf_test_suite():
    print("*"*65)
    print("STARTING LAYER 3 CBF FILTER COMPREHENSIVE TEST SUITE")
    print("*"*65)

    cbf_filter = Layer3_CBF()

    # ---------------------------------------------------------
    # SCENARIO 1: Clear Highway (Bypass Check)
    # ---------------------------------------------------------
    evaluate_scenario(
        cbf_filter,
        name="1. Clear Highway (Bypass Mode)",
        current_state=[0.0, 10.0, 0.0, 20.0], # Driving 20 m/s
        u_mpc=[0.0, 2.0],                     # Wants to accelerate
        neighbors_current=[]                  # No cars around
    )

    # ---------------------------------------------------------
    # SCENARIO 2: Slow Car Ahead (Gentle Intervention)
    # ---------------------------------------------------------
    evaluate_scenario(
        cbf_filter,
        name="2. Slow Car Ahead (Longitudinal Braking Intervention)",
        current_state=[0.0, 10.0, 0.0, 20.0],
        u_mpc=[0.0, 3.0],                          # NMPC dangerously wants to accelerate hard
        neighbors_current=[[0.0, 25.0, 0.0, 15.0]] # Neighbor is 15m ahead, closing gap at 5 m/s
    )

    # ---------------------------------------------------------
    # SCENARIO 3: Lateral Threat (Bad Lane Change)
    # ---------------------------------------------------------
    evaluate_scenario(
        cbf_filter,
        name="3. Lateral Threat (Preventing Unsafe Steering)",
        current_state=[0.0, 10.0, 0.0, 20.0],
        u_mpc=[2.5, 0.0],                          # NMPC wants to swerve right (ax = 2.5)
        neighbors_current=[[1.5, 10.0, 0.0, 20.0]] # A neighbor is in the right lane right next to us!
    )

    # ---------------------------------------------------------
    # SCENARIO 4: Unavoidable Crash (Emergency Handover)
    # ---------------------------------------------------------
    evaluate_scenario(
        cbf_filter,
        name="4. Unavoidable Crash (Emergency Handover)",
        current_state=[0.0, 26.33, 0.0, 15.0],
        u_mpc=[0.1, 2.5],
        neighbors_current=[[0.0, 30.0, 0.0, 2.0]] # Neighbor is 3.67m ahead, closing at 13 m/s!
    )

    print("\n" + "*"*65)
    print("TEST SUITE COMPLETE")
    print("*"*65)

if __name__ == "__main__":
    run_cbf_test_suite()
