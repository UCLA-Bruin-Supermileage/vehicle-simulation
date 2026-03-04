class StraightFlatPowerModel:
    """
    v2: Straight + flat terrain, calibrated from your measured steady-speed point.

    Model:
      F_res(v) = F_roll + F_drag = (Crr*m*g) + (k_drag*v^2)
      P = F_res(v)*v at steady speed

    Calibration inputs (steady + flat):
      - P_cal_W at v_cal_mph  (e.g. 160W @ 19mph)
      - v_top_mph             (e.g. 45mph top speed)

    Drive:
      - Power-limited: F_power = eta*(u*Pmax) / max(v, v_eps)
      - Optional low-speed force cap: F_drive = min(Fmax, F_power) if Fmax provided
      - Braking as force: F_brake = brake * Fbrake_max

    Integration (Euler):
      v <- max(0, v + a*dt)
      x <- x + v*dt
    """

    MPH_TO_MPS = 0.44704

    def __init__(
        self,
        m=136.078,
        g=9.81,
        Crr=0.014,
        eta=0.85,
        v_eps=0.5,

        # --- calibration "new stuff" ---
        P_cal_W=160.0,
        v_cal_mph=19.0,
        v_top_mph=45.0,

        # If you want to override calibration, pass these explicitly:
        Pmax=None,      # W
        k_drag=None,    # N/(m/s)^2

        # Optional low-speed force cap (torque/traction/current limit)
        Fmax=None,      # N

        # Braking force scale
        Fbrake_max=400.0,
    ):
        self.m = float(m)
        self.g = float(g)
        self.Crr = float(Crr)
        self.eta = float(eta)
        self.v_eps = float(v_eps)

        self.Fmax = None if Fmax is None else float(Fmax)
        self.Fbrake_max = float(Fbrake_max)

        # Rolling resistance (constant on flat)
        self.F_roll = self.Crr * self.m * self.g

        # Convert speeds
        v1 = float(v_cal_mph) * self.MPH_TO_MPS
        vtop = float(v_top_mph) * self.MPH_TO_MPS

        # Calibrate k_drag and Pmax unless overridden
        if k_drag is None:
            # From steady-speed point: P_cal = (F_roll + k*v1^2)*v1
            # => k = (P_cal/v1 - F_roll) / v1^2
            F_res_at_v1 = float(P_cal_W) / max(v1, 1e-9)
            self.k_drag = (F_res_at_v1 - self.F_roll) / max(v1 * v1, 1e-9)
            # If user's Crr is so large that this goes negative, clamp at 0
            if self.k_drag < 0.0:
                self.k_drag = 0.0
        else:
            self.k_drag = float(k_drag)

        if Pmax is None:
            # At top speed steady: Pmax = (F_roll + k*vtop^2)*vtop
            F_res_top = self.F_roll + self.k_drag * (vtop * vtop)
            self.Pmax = F_res_top * vtop
        else:
            self.Pmax = float(Pmax)

    @staticmethod
    def clamp(x, lo=0.0, hi=1.0):
        x = float(x)
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def step(self, state, throttle, brake, dt):
        dt = float(dt)

        # clamp inputs
        u = self.clamp(throttle, 0.0, 1.0)
        b = self.clamp(brake, 0.0, 1.0)

        v = float(state.v)

        # Resistive forces
        F_drag = self.k_drag * (v * v)
        F_res = self.F_roll + F_drag

        # Power command & wheel power
        P_cmd = u * self.Pmax                  # W (commanded cap)
        P_wheel = self.eta * P_cmd             # W (after drivetrain eff)

        # Power-limited drive force
        F_power = P_wheel / max(v, self.v_eps)

        # Optional low-speed force cap
        F_drive = F_power if self.Fmax is None else min(self.Fmax, F_power)

        # Braking force opposing motion (only when moving forward)
        F_brake = (b * self.Fbrake_max) if v > 0.0 else 0.0

        # Net force + accel
        F_net = F_drive - F_res - F_brake
        a = F_net / self.m

        # Integrate (Euler)
        v_new = v + a * dt
        if v_new < 0.0:
            v_new = 0.0

        state.a = a
        state.v = v_new
        state.x = float(state.x) + v_new * dt
        state.t = float(state.t) + dt

        return {
            # ----- DRIVER INPUTS -----

            # Throttle command in [0, 1]
            # 0 = no throttle, 1 = full throttle
            "throttle": u,

            # Brake command in [0, 1]
            # 0 = no braking, 1 = max braking force
            "brake": b,


            # ----- CALIBRATED / MODEL PARAMETERS -----

            # Maximum available power of the vehicle (W)
            # This is the power limit inferred from:
            #   - 160 W @ 19 mph (steady, flat)
            #   - 45 mph top speed
            #   - assumed Crr = 0.008
            "Pmax_W": self.Pmax,

            # Aerodynamic drag coefficient in the simplified form:
            #   F_drag = k_drag * v^2
            # Units: N / (m/s)^2
            # Encodes air density, Cd, and frontal area together
            "k_drag_N_per_(m/s)^2": self.k_drag,

            # Constant rolling resistance force on flat ground:
            #   F_roll = Crr * m * g
            # Always opposes motion and does NOT depend on speed
            "F_roll_N": self.F_roll,


            # ----- POWER TERMS -----

            # Power requested by the driver via throttle:
            #   P_cmd = throttle * Pmax
            # This is the "ideal" commanded power BEFORE drivetrain losses
            "power_cmd_W": P_cmd,

            # Actual mechanical power delivered to the wheels:
            #   P_wheel = eta * P_cmd
            # Accounts for drivetrain efficiency (motor, electronics, chain, etc.)
            # This is the power that actually produces tractive force
            "power_wheel_W": P_wheel,


            # ----- FORCE TERMS -----

            # Tractive force implied by available wheel power:
            #   F_power = P_wheel / v
            # (clamped using v_eps at low speed)
            # This is what limits acceleration at higher speeds
            "F_power_N": F_power,

            # Final drive force applied to the vehicle:
            #   F_drive = min(F_power, Fmax) if Fmax exists
            #   F_drive = F_power otherwise
            # This is the force actually pushing the car forward
            "F_drive_N": F_drive,

            # Aerodynamic drag force:
            #   F_drag = k_drag * v^2
            # Grows quadratically with speed and dominates at high speed
            "F_drag_N": F_drag,

            # Total resistive force opposing motion:
            #   F_res = F_roll + F_drag
            # Does NOT include braking force
            "F_res_N": F_res,

            # Braking force applied by the brakes:
            #   F_brake = brake * Fbrake_max
            # Always opposes motion
            "F_brake_N": F_brake,

            # Net longitudinal force on the vehicle:
            #   F_net = F_drive - F_res - F_brake
            # This is the force used in Newton's 2nd law
            "F_net_N": F_net,
        }

