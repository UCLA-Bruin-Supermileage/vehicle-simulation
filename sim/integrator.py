import inspect


def _controller_output(controller, state, track=None):
    params = inspect.signature(controller).parameters
    if track is not None and len(params) >= 2:
        result = controller(state, track)
    else:
        result = controller(state)

    if len(result) == 2:
        throttle, brake = result
        return throttle, brake, {}

    throttle, brake, outputs = result
    return throttle, brake, outputs or {}


def _power_for_energy(outputs):
    if "power_cmd_W" in outputs:
        return outputs["power_cmd_W"]
    if "power_W" in outputs:
        return outputs["power_W"]
    return 0.0


def _history_row(state, throttle, brake, outputs, track=None):
    row = {
        "t": state.t,
        "lap": getattr(state, "lap", 0),
        "s_m": state.s_m,
        "x_m": state.s_m,
        "distance_total_m": getattr(state, "distance_total_m", state.s_m),
        "v_mps": state.v_mps,
        "a_mps2": state.a_mps2,
        "throttle": float(throttle),
        "brake": float(brake),
        "energy_used_J": getattr(state, "energy_used_J", 0.0),
        "energy_used_Wh": getattr(state, "energy_used_J", 0.0) / 3600.0,
        **outputs,
    }
    if track is not None:
        x_track_m, y_track_m = track.xy_at(state.s_m)
        segment = track.segment_at(state.s_m)
        lateral_accel_mps2 = (state.v_mps * state.v_mps) * segment.curvature_1pm
        row.update({
            "track_x_m": x_track_m,
            "track_y_m": y_track_m,
            "segment": segment.name,
            "curvature_1pm": segment.curvature_1pm,
            "lateral_accel_mps2": lateral_accel_mps2,
            "lateral_accel_g": lateral_accel_mps2 / 9.81,
        })
    return row

def run(model, state, controller, T: float, dt: float):
    """
    Generic time integrator.
    - model.step(state, throttle, brake, dt) updates the state in-place
    - controller(state) returns (throttle, brake)
    Returns: list[dict] history
    """
    history = [_history_row(state, 0.0, 0.0, {})]

    while state.t < T:
        old_s = state.s_m
        throttle, brake, controller_outputs = _controller_output(controller, state)
        outputs = model.step(state, throttle, brake, dt) or {}
        outputs = {**controller_outputs, **outputs}
        state.distance_total_m += max(0.0, state.s_m - old_s)
        state.energy_used_J += max(0.0, float(_power_for_energy(outputs))) * float(dt)
        history.append(_history_row(state, throttle, brake, outputs))

    return history


def run_laps(model, track, state, controller, laps: int, dt: float, max_time: float = 3600.0):
    """
    Integrate until the requested lap count is completed.

    The vehicle model remains longitudinal: it advances state.s_m. This runner
    wraps that distance around the track and records track coordinates/segments.
    """
    target_laps = int(laps)
    if target_laps <= 0:
        raise ValueError("laps must be positive")

    history = [_history_row(state, 0.0, 0.0, {}, track)]

    while state.lap < target_laps and state.t < max_time:
        old_s = state.s_m
        throttle, brake, controller_outputs = _controller_output(controller, state, track)
        outputs = model.step(state, throttle, brake, dt) or {}
        outputs = {**controller_outputs, **outputs}

        new_s = state.s_m
        delta_s = max(0.0, new_s - old_s)
        state.distance_total_m += delta_s
        state.energy_used_J += max(0.0, float(_power_for_energy(outputs))) * float(dt)

        while state.s_m >= track.length_m:
            state.s_m -= track.length_m
            state.lap += 1

        history.append(_history_row(state, throttle, brake, outputs, track))

    if state.lap < target_laps:
        raise RuntimeError(f"simulation stopped at max_time={max_time}s before finishing {target_laps} lap(s)")

    return history
