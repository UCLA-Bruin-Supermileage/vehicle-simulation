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
