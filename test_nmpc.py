import time
import numpy as np
from nmpc_layer import Layer2_NMPC

def run_unit_test():
    print("--- NMPC ISOLATED UNIT TEST ---")

    # 1. Initialize the NMPC (N=25, dt=0.2)
    nmpc = Layer2_NMPC(N=25, dt=0.2)

    # 2. Define a Test Scenario: Car is going slow (5 m/s) and needs to change lanes (x = 3.0)
    current_state = [0.0, 0.0, 0.0, 5.0]  # [x, y, vx, vy]
    final_goal = [3.0, 100.0]             # Target lane is x=3.0

    # Mock AI Prediction (Asking for a smooth lane change to x=3.0)
    mock_ego_data = {
        'Pred_Lateral_m': np.linspace(0.0, 3.0, 26).tolist(),
        'Pred_LocalY_m': np.linspace(0.0, 100.0, 26).tolist()
    }
    mock_neighbors = [] # No neighbors for this basic kinematics test

    # 3. Run the Cold Start (First Frame)
    start_time = time.time()
    result1 = nmpc.solve(current_state, final_goal, mock_ego_data, mock_neighbors)
    cold_time = (time.time() - start_time) * 1000

    # 4. Run the Warm Start (Second Frame - 0.2 seconds later)
    # Update state based on the first command
    next_vy = current_state[3] + result1['a_y'][0] * 0.2
    next_x = current_state[0] + result1['a_x'][0] * 0.2
    current_state = [next_x, 0.0, 0.0, next_vy]

    start_time = time.time()
    result2 = nmpc.solve(current_state, final_goal, mock_ego_data, mock_neighbors)
    warm_time = (time.time() - start_time) * 1000

    # 5. Print Formal Results
    print(f"\n[Performance Metrics]")
    print(f"Cold Start Execution Time: {cold_time:.2f} ms")
    print(f"Warm Start Execution Time: {warm_time:.2f} ms")

    print("\n[Planned Trajectory (First 5 steps)]")
    print(f"{'Step':<6} | {'Steering (a_x)':<15} | {'Throttle (a_y)':<15}")
    print("-" * 45)
    for i in range(5):
        print(f"{i:<6} | {result2['a_x'][i]:<15.4f} | {result2['a_y'][i]:<15.4f}")

if __name__ == "__main__":
    run_unit_test()
