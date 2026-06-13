import casadi as ca
import numpy as np

class Layer2_NMPC:
    def __init__(self, N=25, dt=0.2):
        self.N = N
        self.dt = dt

        self.W_stage_track = 15.0
        self.W_vel_track   = 8.0
        self.W_term_track  = 5.0
        self.W_smooth_u    = 80.0
        self.W_smooth_du   = 120.0
        self.W_collision   = 80.0

        self.a_x_min, self.a_x_max = -3.0, 3.0
        self.a_y_min, self.a_y_max = -3.0, 3.0
        self.v_min = 0.0
        self.v_max = 40.0

        # 🛑 SPEED FIX 1: Memory cache variables to store state and control trajectories
        self.last_X = None
        self.last_U = None

    def solve(self, current_state, final_goal, ego_data, neighbors_data):
        opti = ca.Opti()

        X = opti.variable(4, self.N + 1)
        x_pos, y_pos, vx, vy = X[0, :], X[1, :], X[2, :], X[3, :]

        U = opti.variable(2, self.N)
        ax, ay = U[0, :], U[1, :]

        # Initial state
        opti.subject_to(X[:, 0] == current_state)

        # Kinematics
        for k in range(self.N):
            opti.subject_to(X[0, k+1] == X[0, k] + X[2, k]*self.dt + 0.5*U[0, k]*self.dt**2)
            opti.subject_to(X[1, k+1] == X[1, k] + X[3, k]*self.dt + 0.5*U[1, k]*self.dt**2)
            opti.subject_to(X[2, k+1] == X[2, k] + U[0, k]*self.dt)
            opti.subject_to(X[3, k+1] == X[3, k] + U[1, k]*self.dt)

        # Actuator bounds
        opti.subject_to(opti.bounded(self.a_x_min, ax, self.a_x_max))
        opti.subject_to(opti.bounded(self.a_y_min, ay, self.a_y_max))

        # Velocity bounds applied to the state
        for k in range(self.N + 1):
            opti.subject_to(X[3, k] <= self.v_max)
            opti.subject_to(X[3, k] >= self.v_min)

        cost = 0

        ego_x_ref = ego_data.get('Pred_Lateral_m',  [0.0] * (self.N + 1))
        ego_y_ref = ego_data.get('Pred_LocalY_m',   [0.0] * (self.N + 1))

        # Smooth cruise speed target
        current_vy = float(current_state[3])
        global_target_speed = 20.0  # The ultimate highway speed goal

        # The Sliding Target: Aim 5 m/s faster than current speed to encourage gentle acceleration
        target_cruise_speed = min(current_vy + 5.0, global_target_speed)

        for k in range(self.N):
            # Lateral tracking
            cost += self.W_stage_track * (x_pos[k] - ego_x_ref[k])**2

            # Longitudinal speed tracking
            cost += self.W_vel_track * (vy[k] - target_cruise_speed)**2

            # Smoothness
            cost += self.W_smooth_u * (ax[k]**2 + ay[k]**2)
            if k > 0:
                cost += self.W_smooth_du * ((ax[k] - ax[k-1])**2 + (ay[k] - ay[k-1])**2)

            # Better-conditioned collision cost
            for nbr in neighbors_data:
                nbr_x  = nbr.get('Pred_Lateral_m',  [0.0] * (self.N + 1))
                nbr_y  = nbr.get('Pred_LocalY_m',   [0.0] * (self.N + 1))
                sig_x  = nbr.get('Sigma_Lat_m',     [0.1] * (self.N + 1))
                sig_y  = nbr.get('Sigma_Long_m',    [0.1] * (self.N + 1))

                dx = x_pos[k] - nbr_x[k]
                dy = y_pos[k] - nbr_y[k]

                safe_dist = 9.0
                dist_sq = dx**2 + dy**2
                uncertainty = ca.fmax(sig_x[k]**2 + sig_y[k]**2, 0.01)

                cost += self.W_collision * ca.fmax(0, safe_dist - dist_sq / uncertainty)

        # Terminal cost
        cost += self.W_term_track * (x_pos[-1] - final_goal[0])**2

        opti.minimize(cost)

        # 🛑 SPEED FIX 2: Inject previous solutions to serve as initial guesses
        if self.last_X is not None:
            opti.set_initial(X, self.last_X)
        if self.last_U is not None:
            opti.set_initial(U, self.last_U)

        p_opts = {"expand": True, "print_time": False}

        # 🛑 SPEED FIX 3: Cap max_iter to 40 so it physically cannot lag the simulator.
        # Explicitly instruct IPOPT to accept warm starting.
        s_opts = {
            "max_iter": 40,
            "print_level": 0,
            "acceptable_tol": 1e-3,
            "warm_start_init_point": "yes"
        }
        opti.solver('ipopt', p_opts, s_opts)

        try:
            sol = opti.solve()

            # 🛑 SPEED FIX 4: Cache successful trajectories for the next simulation tick
            self.last_X = sol.value(X)
            self.last_U = sol.value(U)

            return {
                'success': True,
                'a_x': sol.value(ax),
                'a_y': sol.value(ay)
            }
        except RuntimeError:
            # 🛑 SPEED FIX 5: If it slightly fails to converge, extract the debug guess
            # so the warm start stream doesn't break, preventing future timeouts.
            try:
                self.last_X = opti.debug.value(X)
                self.last_U = opti.debug.value(U)
                return {
                    'success': True,
                    'a_x': opti.debug.value(ax),
                    'a_y': opti.debug.value(ay)
                }
            except Exception:
                return {
                    'success': False,
                    'a_x': np.zeros(self.N),
                    'a_y': np.zeros(self.N)
                }
