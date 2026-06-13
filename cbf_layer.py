import numpy as np
from scipy.optimize import minimize

class Layer3_CBF:
    def __init__(self, alpha1=1.5, alpha2=1.5):
        self.alpha1 = alpha1
        self.alpha2 = alpha2

        # Bounding Box Physical Margins
        self.d_long = 5.0
        self.d_lat  = 2.0

        # Strict physical actuator limits
        self.a_x_min, self.a_x_max = -3.0, 3.0
        self.a_y_min, self.a_y_max = -6.0, 4.0

    def filter_control(self, current_state, u_mpc, neighbors_current):
        x_ego, y_ego, vx_ego, vy_ego = current_state
        ax_mpc, ay_mpc = u_mpc

        def objective(u):
            return 0.5 * ((u[0] - ax_mpc)**2 + (u[1] - ay_mpc)**2)

        constraints = []

        for nbr in neighbors_current:
            x_nbr, y_nbr, vx_nbr, vy_nbr = nbr

            dp_x = x_nbr - x_ego
            dp_y = y_nbr - y_ego
            dv_x = vx_nbr - vx_ego
            dv_y = vy_nbr - vy_ego

            if abs(dp_x) <= self.d_lat:
                if dp_y > 0:
                    def cbf_long_ahead(u, dp=dp_y, dv=dv_y):
                        b = - ((self.alpha1 + self.alpha2)*dv + self.alpha1*self.alpha2*(dp - self.d_long))
                        return -1.0 * u[1] - b
                    constraints.append({'type': 'ineq', 'fun': cbf_long_ahead})
                else:
                    def cbf_long_behind(u, dp=dp_y, dv=dv_y):
                        b = - ((self.alpha1 + self.alpha2)*(-dv) + self.alpha1*self.alpha2*(-dp - self.d_long))
                        return 1.0 * u[1] - b
                    constraints.append({'type': 'ineq', 'fun': cbf_long_behind})

            elif abs(dp_y) <= self.d_long:
                if dp_x > 0:
                    def cbf_lat_right(u, dp=dp_x, dv=dv_x):
                        b = - ((self.alpha1 + self.alpha2)*dv + self.alpha1*self.alpha2*(dp - self.d_lat))
                        return -1.0 * u[0] - b
                    constraints.append({'type': 'ineq', 'fun': cbf_lat_right})
                else:
                    def cbf_lat_left(u, dp=dp_x, dv=dv_x):
                        b = - ((self.alpha1 + self.alpha2)*(-dv) + self.alpha1*self.alpha2*(-dp - self.d_lat))
                        return 1.0 * u[0] - b
                    constraints.append({'type': 'ineq', 'fun': cbf_lat_left})

        # --- SPEED BYPASS: Skip Heavy Math if no cars are nearby ---
        if not constraints:
            ax_safe = float(np.clip(ax_mpc, self.a_x_min, self.a_x_max))
            ay_safe = float(np.clip(ay_mpc, self.a_y_min, self.a_y_max))
            return {
                "success": True,
                "u_safe": [ax_safe, ay_safe],
                "mode": "CBF_Unconstrained_Bypass",
                "handover_to_rule_based": False
            }

        # 3. Actuator Bounds
        bounds = [
            (self.a_x_min, self.a_x_max),
            (self.a_y_min, self.a_y_max)
        ]

        # 4. Solve QP
        initial_guess = [ax_mpc, ay_mpc]
        res = minimize(objective, initial_guess, method='SLSQP', bounds=bounds, constraints=constraints)

        # 5. Handover Failsafe Logic
        if res.success:
            return {
                "success": True,
                "u_safe": res.x,
                "mode": "CBF_Filtered",
                "handover_to_rule_based": False
            }
        else:
            return {
                "success": False,
                "u_safe": None,
                "mode": "Decision_Blocked",
                "handover_to_rule_based": True
            }
