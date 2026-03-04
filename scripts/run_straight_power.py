# scripts/run_and_visualize.py
import sys, os

from anyio import Path
from datetime import datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.export import save_csv, save_json
from sim.state import VehicleState #the initial state (position, velocity, acceleration, time) of the car 
from sim.integrator import run #the function responsible for updating the state, throttle, and brake of the car 
from models.straight_flat_power import StraightFlatPowerModel
from visualization.track_anim import animate_history_on_track


def controller_full_throttle(state):
    return 1.0, 0.0


def main():
    state = VehicleState()

    # Calibrate from:
    # - steady power point: 160 W @ 19 mph on flat
    # - top speed: 45 mph on flat
    # Assumption to make this solvable: Crr = 0.008
    model = StraightFlatPowerModel(
        m=136.078,
        Crr=0.008,     # <- the assumption we used
        g=9.81,
        eta=0.85,
        v_eps=0.5,

        # calibration inputs
        P_cal_W=160.0,
        v_cal_mph=19.0,
        v_top_mph=45.0,

        # let the model infer these
        Pmax=None,
        k_drag=None,

        # leave force cap off for now; add later if launch is too aggressive
        Fmax=None,
        Fbrake_max=400.0,
    )

    history = run(model, state, controller_full_throttle, T=120.0, dt=0.01)

    # ---- EXPORT ----
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path("outputs") / "straight_flat_power" / stamp
    save_csv(history, out_dir / "telemetry.csv")
    save_json(history, out_dir / "telemetry.json")

    print(f"Saved: {out_dir/'telemetry.csv'}")
    print(f"Saved: {out_dir/'telemetry.json'}")

    # Print calibrated params so you can sanity-check
    print(f"Calibrated Pmax (W): {model.Pmax:.2f}")
    print(f"Calibrated k_drag (N/(m/s)^2): {model.k_drag:.6f}")
    print(f"Rolling force F_roll (N): {model.F_roll:.2f}")

    # ---- VISUALIZE ----
    animate_history_on_track(history)


if __name__ == "__main__":
    main()
