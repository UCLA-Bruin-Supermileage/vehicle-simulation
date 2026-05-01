import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from sim.track import M_TO_FT, OvalTrack


def build_oval_track(long_straight_ft=3300.0, short_straight_ft=660.0, corner_arc_ft=1320.0,
                     n_arc=200, n_line=200):
    track = OvalTrack.from_feet(long_straight_ft, short_straight_ft, corner_arc_ft)
    x_m, y_m = track.sample_xy(points_per_segment=max(n_arc, n_line))
    geom = {
        "R": track.corner_radius_m * M_TO_FT,
        "Lx": track.long_straight_m * M_TO_FT,
        "Ly": track.short_straight_m * M_TO_FT,
        "x_left": track.x_left_m * M_TO_FT,
        "y_bottom": track.y_bottom_m * M_TO_FT,
    }
    return np.array(x_m) * M_TO_FT, np.array(y_m) * M_TO_FT, geom


def map_history_to_bottom_straight(history, geom):
    """
    Backward-compatible mapping for old straight-line histories.
    """
    x_m = np.array([row["x_m"] for row in history], dtype=float)
    s_ft = np.clip(x_m * M_TO_FT, 0.0, geom["Lx"])
    x_track = geom["x_left"] + s_ft
    y_track = np.full_like(x_track, geom["y_bottom"])
    return x_track, y_track


def _history_track_xy(history, track, geom):
    if history and "track_x_m" in history[0] and "track_y_m" in history[0]:
        car_x = np.array([row["track_x_m"] for row in history], dtype=float) * M_TO_FT
        car_y = np.array([row["track_y_m"] for row in history], dtype=float) * M_TO_FT
        return car_x, car_y

    if history and "s_m" in history[0]:
        coords = [track.xy_at(row["s_m"]) for row in history]
        car_x = np.array([xy[0] for xy in coords], dtype=float) * M_TO_FT
        car_y = np.array([xy[1] for xy in coords], dtype=float) * M_TO_FT
        return car_x, car_y

    return map_history_to_bottom_straight(history, geom)


def animate_history_on_track(history, track=None, title="Car on track"):
    if track is None:
        track = OvalTrack.from_feet(3300.0, 660.0, 1320.0)

    track_x_m, track_y_m = track.sample_xy(points_per_segment=200)
    track_x = np.array(track_x_m) * M_TO_FT
    track_y = np.array(track_y_m) * M_TO_FT
    geom = {
        "Lx": track.long_straight_m * M_TO_FT,
        "x_left": track.x_left_m * M_TO_FT,
        "y_bottom": track.y_bottom_m * M_TO_FT,
    }

    car_x, car_y = _history_track_xy(history, track, geom)

    t = np.array([row["t"] for row in history], dtype=float)
    v_mps = np.array([row["v_mps"] for row in history], dtype=float)
    last_idx = len(history) - 1

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(track_x, track_y, linewidth=8)

    car_dot, = ax.plot([], [], marker="o", markersize=10)

    pad = 400
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(track_x.min() - pad, track_x.max() + pad)
    ax.set_ylim(track_y.min() - pad, track_y.max() + pad)
    ax.set_title(title)
    ax.set_xlabel("x (feet)")
    ax.set_ylabel("y (feet)")

    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left")

    def init():
        car_dot.set_data([], [])
        info.set_text("")
        return car_dot, info

    def update(i):
        if i > last_idx:
            i = last_idx

        car_dot.set_data([car_x[i]], [car_y[i]])

        mph = v_mps[i] * 2.23694
        dist_ft = history[i].get("distance_total_m", history[i]["x_m"]) * M_TO_FT
        segment = history[i].get("segment", "")

        info.set_text(
            f"t = {t[i]:.2f} s\n"
            f"v = {v_mps[i]:.2f} m/s ({mph:.1f} mph)\n"
            f"distance = {dist_ft:.0f} ft\n"
            f"segment = {segment}"
        )
        return car_dot, info

    step = 5
    frames = range(0, last_idx + 1, step)

    ani = FuncAnimation(fig, update, frames=frames, init_func=init, interval=50, blit=True)
    plt.show()
