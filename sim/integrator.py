# sim/integrator.py
#integrator.py repeadtedly advances the simulation forward by small chunks of time dt using a while loop, updating
#the simulation for each dt to reflect the current state.t, state.x, state.v, state.a values. 



def run(model, state, controller, T: float, dt: float):
    """
    Generic time integrator.
    - model.step(state, throttle, brake, dt) updates the state in-place
    - controller(state) returns (throttle, brake)
    Returns: list[dict] history
    """
    history = [] #log that store a timeline of what happened in each step---essentially a data table 

    # record initial state as well (optional)
    history.append({ #history log is first given the initial state of the car's outputs 
        "t": state.t, #time 
        "x_m": state.x, #position/distance
        "v_mps": state.v, #velocity 
        "a_mps2": state.a, #acceleration
        "throttle": 0.0, #throttle percentage 
        "brake": 0.0, #brake input 
    })

    while state.t < T: #keep the simulation going until we reach the final simulation time T 
        throttle, brake = controller(state) #controller/driver of the car decides throttle and brake inputs (0-1.0) based on the current state of the car 
        outputs = model.step(state, throttle, brake, dt) or {} #for each dt, the model.step() function is used to modify mutable objects state (t, x, v, a), throttle, and brake 

        row = {
            "t": state.t,
            "x_m": state.x,
            "v_mps": state.v,
            "a_mps2": state.a,
            "throttle": float(throttle),
            "brake": float(brake),
            **outputs, #recorded as a single row for a single dt in history list.
        }
        history.append(row) #for each time t, append outputs time, position, velocity, acceleration, throttle, brake to history log---data collection for visualization 

    return history
