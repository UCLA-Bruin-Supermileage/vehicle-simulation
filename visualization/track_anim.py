# visualization/track_anim.py
#builds the gemoetry of an oval race track for visualization 


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def build_oval_track(long_straight_ft=3300.0, short_straight_ft=660.0, corner_arc_ft=1320.0,
                     n_arc=200, n_line=200):
    # Corner radius from arc length of quarter circle: arc = (pi/2)*R

    R = corner_arc_ft / (np.pi / 2.0)

    Lx = long_straight_ft #the length of a long horizontal straight  = 3300.0 ft
    Ly = short_straight_ft #the length of a short vertical straight = 660.0 ft 

    x_left = -Lx / 2.0 #the leftmost point of the track
    x_right = Lx / 2.0 #the rightmost point of the track
    y_bottom = -(Ly / 2.0 + R) #lowest vertical point of track (from top-down view) 
    y_top = +(Ly / 2.0 + R) #highest vertical point 

    c_bl = (x_left, -(Ly / 2.0)) #center of bottom left corner 
    c_br = (x_right, -(Ly / 2.0)) #center of bottom right corner
    c_tr = (x_right, +(Ly / 2.0)) #center of top right corner 
    c_tl = (x_left, +(Ly / 2.0)) #center of top left corner

    # 1) Bottom straight
    xb = np.linspace(x_left, x_right, n_line)  #x moves from left to right in a line 
    yb = np.full_like(xb, y_bottom) #y stays at vertical position y_bottom

    # 2) Bottom-right corner
    th = np.linspace(-np.pi/2, 0.0, n_arc) #creates a quarter circle 
    xbr = c_br[0] + R * np.cos(th)
    ybr = c_br[1] + R * np.sin(th)

    # 3) Right short straight
    yr = np.linspace(-(Ly/2.0), +(Ly/2.0), n_line)
    xr = np.full_like(yr, x_right + R)

    # 4) Top-right corner
    th = np.linspace(0.0, np.pi/2, n_arc)
    xtr = c_tr[0] + R * np.cos(th)
    ytr = c_tr[1] + R * np.sin(th)

    # 5) Top straight
    xt = np.linspace(x_right, x_left, n_line)
    yt = np.full_like(xt, y_top)

    # 6) Top-left corner
    th = np.linspace(np.pi/2, np.pi, n_arc)
    xtl = c_tl[0] + R * np.cos(th)
    ytl = c_tl[1] + R * np.sin(th)

    # 7) Left short straight
    yl = np.linspace(+(Ly/2.0), -(Ly/2.0), n_line)
    xl = np.full_like(yl, x_left - R)

    # 8) Bottom-left corner
    th = np.linspace(np.pi, 3*np.pi/2, n_arc)
    xbl = c_bl[0] + R * np.cos(th)
    ybl = c_bl[1] + R * np.sin(th)

    X = np.concatenate([xb, xbr, xr, xtr, xt, xtl, xl, xbl])
    Y = np.concatenate([yb, ybr, yr, ytr, yt, ytl, yl, ybl])

    geom = {"R": R, "Lx": Lx, "Ly": Ly, "x_left": x_left, "y_bottom": y_bottom}
    return X, Y, geom


def map_history_to_bottom_straight(history, geom):
    """
    Uses history['x_m'] (meters along straight) and maps it to bottom straight in feet.
    """
    x_m = np.array([row["x_m"] for row in history], dtype=float)
    s_ft = x_m * 3.28084  # meters -> feet

    # clamp to the straight length
    s_ft = np.clip(s_ft, 0.0, geom["Lx"])

    x_track = geom["x_left"] + s_ft
    y_track = np.full_like(x_track, geom["y_bottom"])
    return x_track, y_track


def animate_history_on_track(history, title="Car on bottom long straight (v1)"):
    LONG_FT, SHORT_FT, CORNER_ARC_FT = 3300.0, 660.0, 1320.0
    track_x, track_y, geom = build_oval_track(LONG_FT, SHORT_FT, CORNER_ARC_FT)

    car_x, car_y = map_history_to_bottom_straight(history, geom)

    t = np.array([row["t"] for row in history], dtype=float)
    v_mps = np.array([row["v_mps"] for row in history], dtype=float)

    # end frame when reach end of bottom straight
    reached = np.where((np.array([row["x_m"] for row in history]) * 3.28084) >= geom["Lx"])[0]
    last_idx = int(reached[0]) if len(reached) else (len(history) - 1)

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
        dist_ft = history[i]["x_m"] * 3.28084

        info.set_text(
            f"t = {t[i]:.2f} s\n"
            f"v = {v_mps[i]:.2f} m/s ({mph:.1f} mph)\n"
            f"distance along straight = {dist_ft:.0f} ft / {geom['Lx']:.0f} ft"
        )
        return car_dot, info

    # downsample frames for speed
    step = 5
    frames = range(0, last_idx + 1, step)

    ani = FuncAnimation(fig, update, frames=frames, init_func=init, interval=50, blit=True)
    plt.show()
