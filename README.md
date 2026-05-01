```
SMV-SIMULATION/
│
├── sim/
│   ├── __init__.py
│   ├── state.py          # vehicle state object
│   ├── vehicle.py       # physics models (forces, power, etc)
│   ├── track.py         # track geometry & curvature
│   ├── integrator.py    # time stepping
│   └── telemetry.py     # DAQ-like signals
│
├── models/
│   ├── straight_flat.py     # your current v1 dynamics
│   ├── powertrain.py       # later: motor, battery, etc
│   └── aero.py             # later: drag models
│
├── tracks/
│   └── oval.py             # your NASCAR-style track
│
├── visualization/
│   └── track_anim.py       # matplotlib animation
│
└── scripts/
    └── run_straight.py     # small runner script
```

How to run:
python -m scripts.run_straight

Current lap simulator:
python -m scripts.run_lap --laps 1 --no-plot

Optional visualization:
python -m scripts.run_lap --laps 1

Useful strategy options:
python -m scripts.run_lap --strategy pace --target-lap-time 300 --no-plot
python -m scripts.run_lap --strategy corner-aware --cruise-mph 34 --mu 0.75 --no-plot
python -m scripts.run_lap --strategy full-throttle --no-plot

The lap simulator is still a 1D longitudinal model. The vehicle state uses
`s_m` as distance along the track centerline, and the track maps that distance
to x/y coordinates, segment names, and curvature. This keeps the physics simple
while making full-lap energy and lap-time comparison possible.

The `pace` strategy is the energy-focused baseline. It calculates remaining
distance and remaining target time every timestep, then commands only enough
throttle to stay on the required average speed. Use `--target-lap-time` to set
the schedule it should conserve energy against.
