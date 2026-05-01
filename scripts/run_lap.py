import argparse
from datetime import datetime
from pathlib import Path

from models.straight_flat_power import StraightFlatPowerModel
from sim.export import save_csv, save_json
from sim.integrator import run_laps
from sim.state import VehicleState
from tracks.indy_ms import build_track
from visualization.track_anim import animate_history_on_track


def mph_to_mps(mph):
    return float(mph) * 0.44704


def full_throttle_controller(state, track):
    return 1.0, 0.0


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
    parser.add_argument("--strategy", choices=["corner-aware", "full-throttle"], default="corner-aware")
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
        animate_history_on_track(history, track=track, title=f"{args.strategy} lap")


if __name__ == "__main__":
    main()
