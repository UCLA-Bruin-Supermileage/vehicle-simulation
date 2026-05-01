import argparse
from datetime import datetime
from pathlib import Path

from models.straight_flat_power import StraightFlatPowerModel
from sim.export import save_csv, save_json
from sim.integrator import run_laps
from sim.state import VehicleState
from tracks.indy_ms import build_track


def mph_to_mps(mph):
    return float(mph) * 0.44704


def full_throttle_controller(state, track):
    return 1.0, 0.0


def clamp(x, lo=0.0, hi=1.0):
    return min(max(float(x), lo), hi)


class CornerAwareController:
    """
    Simple endurance baseline: hold a target speed on straights and brake early
    enough to meet the next corner's lateral speed limit.
    """

    def __init__(
        self,
        cruise_mph=34.0,
        mu=0.75,
        brake_accel_mps2=2.2,
        speed_margin_mps=0.5,
        lookahead_m=220.0,
    ):
        self.cruise_mps = mph_to_mps(cruise_mph)
        self.mu = float(mu)
        self.brake_accel_mps2 = float(brake_accel_mps2)
        self.speed_margin_mps = float(speed_margin_mps)
        self.lookahead_m = float(lookahead_m)

    def _min_speed_limit_ahead(self, state, track):
        samples = 30
        v_limit = self.cruise_mps
        for i in range(samples + 1):
            ds = self.lookahead_m * i / samples
            local_limit = track.speed_limit_at(state.s_m + ds, mu=self.mu)
            v_limit = min(v_limit, local_limit)
        return v_limit

    def __call__(self, state, track):
        v = state.v_mps
        v_target = self._min_speed_limit_ahead(state, track)

        if v > v_target + self.speed_margin_mps:
            excess = v - v_target
            brake = min(1.0, excess / max(self.brake_accel_mps2, 1e-6))
            return 0.0, brake

        if v < v_target - self.speed_margin_mps:
            return 0.75, 0.0

        return 0.0, 0.0


class PaceToFinishController:
    """
    Energy-focused strategy: calculate remaining distance and remaining time,
    then command only enough throttle to stay on the required average pace.
    """

    def __init__(
        self,
        model,
        track,
        laps,
        target_lap_time_s,
        mu=0.75,
        lookahead_m=220.0,
        speed_margin_mps=0.25,
        response_time_s=8.0,
        max_accel_mps2=0.35,
        brake_accel_mps2=2.2,
    ):
        self.model = model
        self.track = track
        self.total_distance_m = float(laps) * track.length_m
        self.target_total_time_s = float(laps) * float(target_lap_time_s)
        self.mu = float(mu)
        self.lookahead_m = float(lookahead_m)
        self.speed_margin_mps = float(speed_margin_mps)
        self.response_time_s = float(response_time_s)
        self.max_accel_mps2 = float(max_accel_mps2)
        self.brake_accel_mps2 = float(brake_accel_mps2)

    def _min_speed_limit_ahead(self, state, track):
        samples = 30
        v_limit = float("inf")
        for i in range(samples + 1):
            ds = self.lookahead_m * i / samples
            local_limit = track.speed_limit_at(state.s_m + ds, mu=self.mu)
            v_limit = min(v_limit, local_limit)
        return v_limit

    def _required_pace_mps(self, state):
        remaining_distance = max(0.0, self.total_distance_m - state.distance_total_m)
        remaining_time = self.target_total_time_s - state.t
        if remaining_time <= 0.0:
            return float("inf")
        return remaining_distance / remaining_time

    def _pace_state(self, state):
        remaining_distance = max(0.0, self.total_distance_m - state.distance_total_m)
        remaining_time = self.target_total_time_s - state.t
        if remaining_time <= 0.0:
            required_pace = float("inf")
        else:
            required_pace = remaining_distance / remaining_time
        return remaining_distance, remaining_time, required_pace

    def _throttle_for_accel(self, state, accel_mps2):
        v = max(state.v_mps, self.model.v_eps)
        f_drag = self.model.k_drag * (state.v_mps * state.v_mps)
        f_res = self.model.F_roll + f_drag
        f_needed = f_res + self.model.m * max(0.0, accel_mps2)
        power_needed = f_needed * v
        if self.model.eta <= 0.0 or self.model.Pmax <= 0.0:
            return 0.0
        return clamp(power_needed / (self.model.eta * self.model.Pmax))

    def __call__(self, state, track):
        remaining_distance, remaining_time, required_pace = self._pace_state(state)
        corner_limit = self._min_speed_limit_ahead(state, track)
        v_target = min(required_pace, corner_limit)
        v = state.v_mps
        telemetry = {
            "remaining_distance_m": remaining_distance,
            "remaining_time_s": remaining_time,
            "required_pace_mps": required_pace,
            "target_speed_mps": v_target,
            "target_speed_mph": v_target * 2.23694,
            "corner_speed_limit_mps": corner_limit,
        }

        if v > v_target + self.speed_margin_mps:
            if v_target < required_pace - self.speed_margin_mps:
                excess = v - v_target
                return 0.0, clamp(excess / max(self.brake_accel_mps2, 1e-6)), telemetry
            return 0.0, 0.0, telemetry

        if v < v_target - self.speed_margin_mps:
            speed_error = v_target - v
            accel = min(self.max_accel_mps2, speed_error / max(self.response_time_s, 1e-6))
            return self._throttle_for_accel(state, accel), 0.0, telemetry

        return self._throttle_for_accel(state, 0.0), 0.0, telemetry


def build_model():
    return StraightFlatPowerModel(
        m=136.078,
        Crr=0.008,
        g=9.81,
        eta=0.85,
        v_eps=0.5,
        P_cal_W=160.0,
        v_cal_mph=19.0,
        v_top_mph=45.0,
        Pmax=None,
        k_drag=None,
        Fmax=None,
        Fbrake_max=400.0,
    )


def summarize(history, track):
    last = history[-1]
    lap_time = last["t"]
    energy_Wh = last["energy_used_Wh"]
    avg_speed_mps = track.length_m / lap_time
    return {
        "lap_time_s": lap_time,
        "lap_time_min": lap_time / 60.0,
        "energy_Wh": energy_Wh,
        "avg_speed_mph": avg_speed_mps * 2.23694,
        "top_speed_mph": max(row["v_mps"] for row in history) * 2.23694,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--strategy", choices=["pace", "corner-aware", "full-throttle"], default="pace")
    parser.add_argument("--target-lap-time", type=float, default=300.0, help="Target seconds per lap for pace strategy.")
    parser.add_argument("--cruise-mph", type=float, default=34.0)
    parser.add_argument("--mu", type=float, default=0.75)
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    track = build_track()
    state = VehicleState()
    model = build_model()

    if args.strategy == "full-throttle":
        controller = full_throttle_controller
    elif args.strategy == "pace":
        controller = PaceToFinishController(
            model=model,
            track=track,
            laps=args.laps,
            target_lap_time_s=args.target_lap_time,
            mu=args.mu,
        )
    else:
        controller = CornerAwareController(cruise_mph=args.cruise_mph, mu=args.mu)

    history = run_laps(model, track, state, controller, laps=args.laps, dt=args.dt)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path("outputs") / "lap" / args.strategy / stamp
    save_csv(history, out_dir / "telemetry.csv")
    save_json(history, out_dir / "telemetry.json")

    stats = summarize(history, track)
    print(f"Saved: {out_dir / 'telemetry.csv'}")
    print(f"Saved: {out_dir / 'telemetry.json'}")
    print(f"Lap time: {stats['lap_time_s']:.2f}s ({stats['lap_time_min']:.2f} min)")
    print(f"Energy: {stats['energy_Wh']:.2f} Wh")
    print(f"Average speed: {stats['avg_speed_mph']:.1f} mph")
    print(f"Top speed: {stats['top_speed_mph']:.1f} mph")
    print(f"Track length: {track.length_m:.1f} m ({track.length_m * 3.28084:.0f} ft)")

    if not args.no_plot:
        from visualization.track_anim import animate_history_on_track

        animate_history_on_track(history, track=track, title=f"{args.strategy} lap")


if __name__ == "__main__":
    main()
