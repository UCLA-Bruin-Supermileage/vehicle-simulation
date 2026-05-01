from sim.track import OvalTrack


def build_track():
    """
    Approximate Indianapolis-style oval dimensions.

    These are centerline-style working values for simulation, not survey data.
    """
    return OvalTrack.from_feet(
        long_straight_ft=3300.0,
        short_straight_ft=660.0,
        corner_arc_ft=1320.0,
        name="indy_ms_approx",
    )


TRACK = build_track()
