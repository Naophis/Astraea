import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import signal

dt = 0.001
m = 0.015
Tc = 0.001

# K = 175
# K = 135
# K = 135
# K = 200

list_K_x = [1]


# list_K_y = [1, 1, 1, 2]

# list_K_y = [10, 20, 20]


class Slalom:
    base_time = 0
    v = 0
    rad = 0
    ang = 0
    Et = 0
    limit_time_count = 0
    base_alpha = 0
    pow_n = 0
    start_theta = 0
    res = {}
    end_pos = {}
    start_offset = 0
    end_offset = 0
    base_ang = 0
    type = ""
    start_offset_list = []
    end_offset_list = []
    turn_offset = {"x": 0, "y": 0}
    cell_size = 90
    half_cell_size = 45
    slip_gain = 50
    K = 1
    list_K_y = []

    def __init__(self, v, rad, n, ang, end_pos, slip_gain, type, K, list_K_y, method="euler", method_w="euler", method_time="euler"):
        self.v = v
        self.rad = rad
        self.ang = ang * math.pi / 180
        self.base_ang = ang
        self.pow_n = n
        self.end_pos = end_pos
        self.type = type
        self.slip_gain = slip_gain
        self.K = K
        self.list_K_y = list_K_y
        self.method = method
        self.method_w = method_w
        self.method_time = method_time

        dx = 0.0001/64

        def safe_integrand(x, n):
            if abs(x) >= 1: return 0
            return np.exp(1) * np.exp(-1 / (1 - x**n))

        def safe_integrand_array(x, n):
            result = np.zeros_like(x)
            valid_indices = (np.abs(x) < 1)
            result[valid_indices] = np.exp(
                1)*np.exp(-1 / (1 - (x[valid_indices])**n))
            return result
        
        if self.method_time == "rk4":
            # RK4 Integration for Et from 0 to 1
            # dy/dx = safe_integrand(x, n)
            # y(0) = 0, find y(1)
            
            y = 0
            x = 0
            step_size = dx
            steps = int(1 / step_size)
            
            for _ in range(steps):
                k1 = safe_integrand(x, n)
                k2 = safe_integrand(x + step_size/2, n)
                k3 = safe_integrand(x + step_size/2, n)
                k4 = safe_integrand(x + step_size, n)
                
                y += (step_size / 6) * (k1 + 2*k2 + 2*k3 + k4)
                x += step_size
            
            self.Et = y
        else:
            # Original Trapezoidal/Euler method using numpy
            x_values_corrected = np.linspace(0, 1, int(1 / dx))
            self.Et = np.trapz(safe_integrand_array(
                x_values_corrected, n), x_values_corrected)
            
        print(f"Et ({self.method_time}): {self.Et}")
        self.base_alpha = v / rad

    def set_cell_size(self, size):
        self.cell_size = size
        self.half_cell_size = size / 2

    def calc_base_time(self):
        c = 0
        t1 = 0
        tmp_dt = 0.001 / 64

        base_time = (self.rad * self.ang)/(2 * self.v *self.Et )
        self.base_time = base_time
        self.limit_time_count = base_time * 2 / dt
        return

        # while c < 10000000:
        #     t1 = t1 + tmp_dt
        #     if (2 * self.v / self.rad * self.Et * t1) >= self.ang:
        #         self.base_time = t1
        #         self.limit_time_count = t1 * 2 / dt
        #         print(f"Base time: {self.base_time}")
        #         print(f"Limit time count: {self.limit_time_count}")
        #         print(f"test time: {(self.rad * self.ang)/(2 * self.v *self.Et )}")
        #         return
        #     c = c + 1

    def get_alpha(self, t):
        return self.base_alpha * self.calc_neipire(t, self.base_time, self.pow_n)

    def integrate_w(self, t, w_curr, dt):
        # Integrate alpha to get next w using selected method
        if self.method_w == "rk4":
            k1 = self.get_alpha(t)
            k2 = self.get_alpha(t + dt/2)
            k3 = self.get_alpha(t + dt/2)
            k4 = self.get_alpha(t + dt)
            w_next = w_curr + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
            return w_next
        else:
            # Euler
            alpha = self.get_alpha(t)
            w_next = w_curr + alpha * dt
            return w_next

    def calc(self, start_ang):
        res = {}
        res["v"] = np.array([])
        res["x"] = np.array([])
        res["y"] = np.array([])
        res["alpha"] = np.array([])
        res["w"] = np.array([])
        res["w2"] = np.array([])
        res["vx"] = np.array([])
        res["vy"] = np.array([])
        res["acc_y"] = np.array([])
        res["ang"] = np.array([])
        
        # State: [x, y, theta]
        # w is calculated separately
        
        state = {
            "x": 0,
            "y": 0,
            "theta": start_ang * math.pi / 180,
            "w": 0 
        }
        
        for i in range(1, int(self.limit_time_count + 1)):
            t = dt * (i - 1) # Start of interval
            
            # Store w_start for RK4 position integration
            w_start = state["w"]
            
            # Update w for the next step
            w_next = self.integrate_w(t, w_start, dt)

            if self.method == "rk4":
                # Pass w_start to step function, it will integrate w internally for intermediate steps
                state = self.step_rk4_calc(t, state, dt, w_start)
            else:
                # Euler uses w_next (Backward Euler style for theta update)
                state = self.step_euler_calc(t, state, dt, w_next)

            # Force update w in state to w_next (as RK4 step might return w_start)
            state["w"] = w_next

            vx = self.v * math.cos(self.start_theta + state["theta"])
            vy = self.v * math.sin(self.start_theta + state["theta"])
            
            # Recalculate alpha/w2 for logging (these are functions of t + dt)
            tmp_alpha = self.get_alpha(t + dt)
            # tmp_w2 is just for logging, maybe analytical?
            tmp_w2 = self.base_alpha * self.calc_neipire_w(t + dt, self.base_time, self.pow_n) * math.exp(1)

            res["x"] = np.append(res["x"], state["x"])
            res["y"] = np.append(res["y"], state["y"])
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], state["w"])
            res["w2"] = np.append(res["w2"], tmp_w2)
            res["v"] = np.append(res["v"], self.v/1000)
            res["vx"] = np.append(res["vx"], vx/1000)
            res["vy"] = np.append(res["vy"], vy/1000)
            res["acc_y"] = np.append(res["acc_y"], (self.v * state["w"]/1000))
            res["ang"] = np.append(res["ang"], state["theta"])

        self.res = res
        return res

    def step_euler_calc(self, t, state, dt, w_next):
        # Euler step for Position
        # x_{n+1} = x_n + v * cos(theta_n) * dt
        # theta_{n+1} = theta_n + w_n * dt
        
        # Note: w_next is the w value at the end of the interval (t+dt)
        # Original code used w_next for theta update.
        
        w = w_next
        theta = state["theta"]
        
        new_theta = theta + w * dt
        
        new_x = state["x"] + self.v * math.cos(self.start_theta + new_theta) * dt
        new_y = state["y"] + self.v * math.sin(self.start_theta + new_theta) * dt
        
        return {"x": new_x, "y": new_y, "theta": new_theta, "w": w}

    def step_rk4_calc(self, t, state, dt, w_start):
        # State vector y = [x, y, theta]
        # dy/dt = f(t, y)
        # dx/dt = v * cos(start_theta + theta)
        # dy/dt = v * sin(start_theta + theta)
        # dtheta/dt = w(t)
        
        # We need w(t) at intermediate points.
        # w is integrated separately.
        # w_start is w at time t.
        
        def get_w_at_offset(offset):
            # Integrate w from t to t + offset
            return self.integrate_w(t, w_start, offset)
            
        def derivs(t_curr, y_curr):
            x, y, theta = y_curr
            # t_curr is t + offset
            offset = t_curr - t
            w = get_w_at_offset(offset)
            
            dxdt = self.v * math.cos(self.start_theta + theta)
            dydt = self.v * math.sin(self.start_theta + theta)
            dthetadt = w
            return np.array([dxdt, dydt, dthetadt])

        y0 = np.array([state["x"], state["y"], state["theta"]])
        
        k1 = derivs(t, y0)
        k2 = derivs(t + dt/2, y0 + dt/2 * k1)
        k3 = derivs(t + dt/2, y0 + dt/2 * k2)
        k4 = derivs(t + dt, y0 + dt * k3)
        
        y_next = y0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        
        return {"x": y_next[0], "y": y_next[1], "theta": y_next[2], "w": w_start}

    def calc_slip(self, start_ang):
        res = {}
        res["x"] = np.array([])
        res["y"] = np.array([])
        res["alpha"] = np.array([])
        res["w"] = np.array([])
        res["v"] = np.array([])
        res["vx"] = np.array([])
        res["vy"] = np.array([])
        res["beta"] = np.array([])
        res["acc_y"] = np.array([])
        res["w2"] = np.array([])
        res["ang"] = np.array([])
        
        # Initial state
        state = {
            "x": 0,
            "y": 0,
            "theta": start_ang * math.pi / 180,
            "vx": self.v / 1000,
            "vy": 0,
            "beta": 0,
            "w": 0 
        }
        
        # For Euler compatibility, we need to track 's' (integral of error)
        self.s_err = 0 

        for i in range(1, int(self.limit_time_count + 1)):
            t = dt * (i - 1) # Start of interval
            
            # Store w_start for RK4
            w_start = state["w"]
            
            # Calculate next w
            w_next = self.integrate_w(t, w_start, dt)
            
            if self.method == "rk4":
                # Pass w_start to step function, it will integrate w internally for intermediate steps
                state = self.step_rk4_slip(t, state, dt, w_start)
            else:
                state = self.step_euler_slip(t, state, dt, w_next) # Euler uses w_next (Backward Euler style)

            # Update w in state
            state["w"] = w_next

            # Logging
            tmp_v = np.sqrt(state["vx"] ** 2 + state["vy"] ** 2)
            tmp_alpha = self.get_alpha(t + dt)
            
            res["v"] = np.append(res["v"], tmp_v)
            res["vx"] = np.append(res["vx"], state["vx"])
            res["vy"] = np.append(res["vy"], state["vy"])

            res["x"] = np.append(res["x"], state["x"])
            res["y"] = np.append(res["y"], state["y"])
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], state["w"])
            res["w2"] = np.append(res["w2"], state["w"])
            res["acc_y"] = np.append(res["acc_y"], (tmp_v * state["w"]))
            res["beta"] = np.append(res["beta"], state["beta"])
            res["ang"] = np.append(res["ang"], state["theta"])

        self.res = res
        return res

    def step_euler_slip(self, t, state, dt, w_next):
        # Original logic used w_next (tmp_w calculated at current time i*dt)
        
        vx = state["vx"]
        vy = state["vy"]
        beta = state["beta"]
        theta = state["theta"]
        
        # Control inputs
        err = self.v / 1000 - np.sqrt(vx ** 2 + vy ** 2)
        self.s_err = self.s_err + err
        Fx = 100.0 * err + 0.01 * self.s_err
        Fx = 0 
        
        v2 = np.sqrt(vx ** 2 + vy ** 2)
        tmpK = np.interp(v2 * 1000, list_K_x, self.list_K_y)
        Fy = -tmpK * beta
        
        # Dynamics
        # Original used old_w (w_prev) for acceleration
        w_prev = state["w"] # This is w at time t (start of interval)
        
        ax = Fx / m + w_prev * vy
        ay = Fy / m - w_prev * vx
        
        new_vy = vy + ay * dt
        new_vx = vx + ax * dt
        
        tmp_v = np.sqrt(new_vx ** 2 + new_vy ** 2)
        
        # Position update
        new_x = state["x"] + tmp_v * 1000 * np.cos(self.start_theta + theta) * dt
        new_y = state["y"] + tmp_v * 1000 * np.sin(self.start_theta + theta) * dt
        
        # Beta update
        # Uses w_next (current w)
        new_beta = (beta / dt - w_next) / (1.0 / dt + self.K / tmp_v)
        
        delta_beta = new_beta - beta
        new_theta = theta + w_next * dt + delta_beta
        
        return {
            "x": new_x, "y": new_y, "theta": new_theta, 
            "vx": new_vx, "vy": new_vy, "beta": new_beta, "w": w_next
        }

    def step_rk4_slip(self, t, state, dt, w_start):
        # State: [x, y, theta, vx, vy, beta, s_err]
        
        def get_w_at_offset(offset):
            # Integrate w from t to t + offset
            return self.integrate_w(t, w_start, offset)
        
        def derivs(t_curr, y_curr):
            x, y, theta, vx, vy, beta, s_err = y_curr
            
            offset = t_curr - t
            w = get_w_at_offset(offset)
            
            v_curr = np.sqrt(vx**2 + vy**2)
            if v_curr < 1e-6: v_curr = 1e-6 
            
            err = self.v / 1000 - v_curr
            Fx = 100.0 * err + 0.01 * s_err
            Fx = 0 
            
            tmpK = np.interp(v_curr * 1000, list_K_x, self.list_K_y)
            Fy = -tmpK * beta
            
            ax = Fx / m + w * vy
            ay = Fy / m - w * vx
            
            dbetadt = -w - (self.K / v_curr) * beta
            dthetadt = w + dbetadt
            
            dxdt = v_curr * 1000 * np.cos(self.start_theta + theta)
            dydt = v_curr * 1000 * np.sin(self.start_theta + theta)
            
            return np.array([dxdt, dydt, dthetadt, ax, ay, dbetadt, err])

        y0 = np.array([state["x"], state["y"], state["theta"], state["vx"], state["vy"], state["beta"], self.s_err])
        
        k1 = derivs(t, y0)
        k2 = derivs(t + dt/2, y0 + dt/2 * k1)
        k3 = derivs(t + dt/2, y0 + dt/2 * k2)
        k4 = derivs(t + dt, y0 + dt * k3)
        
        y_next = y0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        
        self.s_err = y_next[6]
        
        return {
            "x": y_next[0], "y": y_next[1], "theta": y_next[2], 
            "vx": y_next[3], "vy": y_next[4], "beta": y_next[5], "w": 0 # w updated outside
        }


    def calcnormal(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
        res["alpha"] = np.array([0])
        res["w"] = np.array([0])
        tmp_w = 0
        tmp_theta = 0
        tmp_x = 0
        tmp_y = 0
        state = 0
        alpha = 2 * self.v * self.v / (self.rad * self.rad * self.ang / 3)
        while True:
            tmp_alpha = 0
            if state == 0:
                tmp_alpha = alpha
            elif state == 1:
                tmp_alpha = 0
            elif state == 2:
                tmp_alpha = -alpha

            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt

            tmp_x = tmp_x + self.v * \
                math.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + self.v * \
                math.sin(self.start_theta + tmp_theta) * dt

            if state == 0:
                if tmp_theta > (self.ang / 3):
                    state = 1
            elif state == 1:
                if tmp_theta > (self.ang * 2 / 3):
                    state = 2
            elif state == 2:
                if tmp_w < 0:
                    state = 3
                if tmp_theta > (self.ang):
                    state = 3

            if state == 3:
                break

            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)

        self.res = res
        return res

    def calc_slip_normalturn(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
        res["alpha"] = np.array([])
        res["w"] = np.array([])
        res["v"] = np.array([])
        res["vx"] = np.array([])
        res["vy"] = np.array([])
        res["beta"] = np.array([])
        res["acc_y"] = np.array([])
        tmp_w = 0
        tmp_theta = start_ang * math.pi / 180
        tmp_x = 0
        tmp_y = 0
        slip_theta = 0
        ax = 0
        ay = 0
        vx = self.v / 1000
        vy = 0
        Fx = 0
        Fy = 0
        beta = 0
        s = 0
        delta_beta = 0
        old_beta = 0
        state = 0
        alpha = 2 * self.v * self.v / (self.rad * self.rad * self.ang / 3)
        while True:
            tmp_alpha = 0
            if state == 0:
                tmp_alpha = alpha
            elif state == 1:
                tmp_alpha = 0
            elif state == 2:
                tmp_alpha = -alpha

            old_w = tmp_w

            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt + delta_beta
            # Fx = 0

            err = self.v / 1000 - np.sqrt(vx ** 2 + vy ** 2)
            s = s + err
            Fx = 100.0 * err + 0.01 * s

            Fx = 0
            v2 = np.sqrt(vx ** 2 + vy ** 2)
            tmpK = np.interp(v2 * 1000, list_K_x, self.list_K_y)
            Fy = -tmpK * beta
            # print(Fy, tmpK, v2)
            ax = Fx / m + old_w * vy
            ay = Fy / m - old_w * vx

            vy = vy + ay * dt
            vx = vx + ax * dt

            tmp_v = np.sqrt(vx ** 2 + vy ** 2)

            tmp_x = tmp_x + tmp_v * 1000 * \
                np.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + tmp_v * 1000 * \
                np.sin(self.start_theta + tmp_theta) * dt

            # tmp_x = tmp_x + vx * 1000 * dt
            # tmp_y = tmp_y + vy * 1000 * dt

            res["v"] = np.append(res["v"], tmp_v)
            res["vx"] = np.append(res["vx"], vx)
            res["vy"] = np.append(res["vy"], vy)

            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)
            res["acc_y"] = np.append(res["acc_y"], (tmp_v * tmp_w))
            old_beta = beta
            beta = np.arctan2(vy, vx)
            beta = (old_beta / dt - tmp_w) / (1.0 / dt + K / tmp_v)
            res["beta"] = np.append(res["beta"], beta)

            delta_beta = beta - old_beta

            if state == 0:
                if tmp_theta > (self.ang / 3):
                    state = 1
            elif state == 1:
                if tmp_theta > (self.ang * 2 / 3):
                    state = 2
            elif state == 2:
                if tmp_w < 0:
                    state = 3
                if tmp_theta > (self.ang):
                    state = 3

            if state == 3:
                break


        self.res = res

        return res

    def calc_neipire(self, t, s, N):
        if t <= 0: return 0
        t = t / s
        if t >= 2: return 0
        
        try:
            z = math.pow(t - 1, N)
            Q = 1
            P = math.exp(-1 / (1 - z))
            res = -N * P / ((Q - z) * (Q - z)) * \
                math.pow(t - 1, N - 1) / s * math.exp(1)
            return res
        except ZeroDivisionError:
            return 0
        except ValueError:
            return 0
        # if t < s/2:
        #     z = 1
        #     t = t / s
        #     t_z = math.fabs(t - z)
        #     P = math.pow(t_z, N - z)
        #     Q = P * t_z
        #     res = -N * P / ((Q - z) * (Q - z)) * \
        #         (math.exp(z + z / (Q - z)) / s)
        #     return res
        # else:
        #     # 折り返す
        #     z = 1
        #     t = 1 - t / s
        #     if (t-z) < 0:
        #         return 0
        #     P = math.pow((t - z), N - z)
        #     Q = P * (t - z)
        #     res = -N * P / ((Q - z) * (Q - z)) * \
        #         (math.exp(z + z / (Q - z)) / s)
        #     if t == 0:
        #         return 0
        #     return res

    def calc_neipire_w(self, t, s, N):
        t = t / s
        if t <= 0 or t >= 2:
            return 0
        # if (t-1) < 0:
        #     return 0
        try:
            res = math.exp(-1/(1-math.pow(t-1, N)))
        except ZeroDivisionError:
            return 0
        return res

    def calc_offset_dist(self, start_pos_x, start_pos_y, type, offset):
        a = math.sin(self.ang)
        b = math.cos(self.ang)
        if self.ang == 0:
            a = 1
            b = 0
        end_x = self.res["x"][-1]
        end_y = self.res["y"][-1]

        self.end_offset = (self.end_pos["y"] - end_y - start_pos_y[0]) / a
        self.start_offset = (self.end_pos["x"] - end_x) - self.end_offset * b

        # start_offset=0
        # end_offset=0
        end_offset = (self.end_pos["y"] - end_y) / a
        start_offset = (self.end_pos["x"] - end_x) - end_offset * b
        prev_path_x = [0, 0]
        prev_path_y = [0, 0]
        after_path_x = [0, 0]
        after_path_y = [0, 0]

        if self.type == "normal":
            end_offset = (self.end_pos["y"] - end_y)
            start_offset = (self.end_pos["x"] - end_x)
            prev_path_x = [0, start_offset]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "large":
            end_offset = (self.end_pos["y"] - end_y - offset["after"])
            start_offset = (self.end_pos["x"] - end_x + offset["prev"])

            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "orval":
            pass
        elif self.type == "dia45":
            end_offset = (self.end_pos["y"] - end_y) / a
            start_offset = (
                self.end_pos["x"] - end_x + offset["prev"]) - end_offset * b
            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "dia135":
            end_offset = (self.end_pos["y"] - end_y) / a
            start_offset = (
                self.end_pos["x"] - end_x + offset["prev"]) - end_offset * b
            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]

        elif self.type == "dia45_2":

            start_offset = (self.half_cell_size - end_x) / \
                a + offset["prev_dia"]
            end_offset = (self.cell_size - end_y) - \
                (start_offset - offset["prev_dia"]) * a

            prev_path_x = [-offset["prev_dia"] * a,
                           (start_offset - offset["prev_dia"]) * b]
            prev_path_y = [-offset["prev_dia"] * a,
                           (start_offset - offset["prev_dia"]) * a]
            after_path_x = [end_x, end_x]
            after_path_y = [end_y, end_y + end_offset]

            pass

        elif self.type == "dia135_2":

            start_offset = (self.cell_size - end_y) / b + offset["prev"]
            end_offset = math.fabs(
                self.half_cell_size + end_x + abs((start_offset - offset["prev"]) * a))

            prev_path_x = [-offset["prev_dia"] * a,
                           abs((start_offset - offset["prev_dia"]) * a)]
            prev_path_y = [-offset["prev_dia"] * a,
                           abs((start_offset - offset["prev_dia"]) * b)]
            after_path_x = [end_x, end_x - end_offset]
            after_path_y = [end_y, end_y]

            pass
        elif self.type == "dia90":
            self.half_cell_size = 0
            end_offset = (self.cell_size / math.sqrt(2) - end_y)
            start_offset = (self.cell_size / math.sqrt(2) -
                            end_x) + offset["prev_dia"]

            prev_path_x = [-offset["prev_dia"],
                           start_offset - offset["prev_dia"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]

            # prev_path_x = [-offset["prev"], start_offset - offset["prev"]]

        after_path_x2 = [after_path_x[0] + prev_path_x[1],
                         after_path_x[1] + prev_path_x[1]]
        after_path_y2 = [after_path_y[0] + prev_path_y[1],
                         after_path_y[1] + prev_path_y[1]]
        # print(after_path_x)
        # print(after_path_y)
        res = {}

        res["turn_offset_x"] = prev_path_x[1]  # ターンの原点x
        res["turn_offset_y"] = prev_path_y[1]  # ターンの原点y

        res["prev_path_x"] = prev_path_x  # [x0, x1]
        res["prev_path_y"] = prev_path_y  # [y0, y1]
        res["after_path_x"] = after_path_x
        res["after_path_y"] = after_path_y
        res["after_path_x2"] = after_path_x2
        res["after_path_y2"] = after_path_y2
        res["prev_dist"] = self.calc_dist(prev_path_x, prev_path_y)
        res["after_dist"] = self.calc_dist(after_path_x, after_path_y)
        return res

    def calc_dist(self, list_x, list_y):
        d2 = (list_x[1] - list_x[0]) ** 2 + (list_y[1] - list_y[0]) ** 2
        if d2 > 0:
            return math.sqrt(d2)
        return 0
