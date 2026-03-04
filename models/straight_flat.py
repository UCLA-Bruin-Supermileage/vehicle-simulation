# models/straight_flat.py

class StraightFlatModel:
    """
    v1: Straight + flat terrain
    - Power-limited drive force: F = eta * P / max(v, v_eps)
    - Rolling resistance only: Frr = Crr * m * g
    """

    def __init__(self, m=136.1, Crr=0.014, g=9.81, eta=0.85, Pmax=1200.0, v_eps=0.5):
        self.m = float(m) 
        self.Crr = float(Crr) #Coefficient for rolling resistance---the amount of force required to make a tire roll, opposing the car's motion 
        self.g = float(g) #gravity constant 
        self.eta = float(eta) #eta = efficiency---the fraction of power that actually turns into forward motion 
        self.Pmax = float(Pmax) #Maximum power available from the drivetrain 
        self.v_eps = float(v_eps) #a small "safety speed"

        self.Frr = self.Crr * self.m * self.g  # constant on flat v1
                                               #Frr = rolling resistance force 
                                               #m = mass in kg 


#Step function is used to update the car's info (state, throttle, brake) for every tiny change in time dt as it moves through the track 
    #state = the car's current position, velocity, and time 
    #throttle = how much pressure is applied to the accelerator 
    #braking input
    #dt = small time increment
    
    def step(self, state, throttle, brake, dt): 
        # clamp inputs
        u = min(max(float(throttle), 0.0), 1.0) #converts throttle to a float & makes sure can't go below 0 or above 1 
        # brake currently ignored in v1 dynamics, but kept for interface
        _b = min(max(float(brake), 0.0), 1.0) #brake input is between 0 and 1 

        # power available
        P = u * self.Pmax  # Watts---throttle, u, controls how much of the max power produced by drivetrain is actually used for a single dt 

        # drive force (power-limited)
        F_drive = (self.eta * P) / max(state.v, self.v_eps) # Force = power * velocity ---velocity cannot exceed v_eps and max power is limited by energy efficiency 

        # net force
        F_net = F_drive - self.Frr #the net force applied on the car
                                   #F_drive = driving force provided by rotation of car's wheels, inducing friction as torque  
                                   #self.Frr = rolling resistance force due to continuous deformation of car wheels as they roll

        # integrate
        state.a = F_net / self.m    #acceleration of the car 
        state.v = max(0.0, state.v + state.a * dt)  #velocity of the car---always moving right, thus can't be <= 0. 
        state.x = state.x + state.v * dt #position of the car---initial position state.x + tiny displacement for tiny time increment dt 
        state.t = state.t + dt #updating time on sim 

        # return extra telemetry fields to log
        return {
            "power_W": P,
            "F_drive_N": F_drive,
            "F_rr_N": self.Frr,
        }
