# sim/export.py

import csv
import json
from pathlib import Path
from typing import List, Dict, Any

def ensure_parent_dir(path: str | Path) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    return p

def save_json(history: List[Dict[str, Any]], path: str | Path) -> None:
    p = ensure_parent_dir(path)
    with p.open("w", encoding="utf-8") as f:
        json.dump(history, f, indent=2)

def save_csv(history: List[Dict[str, Any]], path: str | Path) -> None:
    """
    Writes a rectangular CSV by using the union of all keys across rows.
    Missing values become empty cells.
    """
    p = ensure_parent_dir(path)
    if not history:
        raise ValueError("history is empty")

    # union of keys (stable order: common keys first if present)
    preferred = [
        "t",
        "lap",
        "s_m",
        "x_m",
        "distance_total_m",
        "track_x_m",
        "track_y_m",
        "segment",
        "curvature_1pm",
        "lateral_accel_mps2",
        "lateral_accel_g",
        "remaining_distance_m",
        "remaining_time_s",
        "required_pace_mps",
        "target_speed_mps",
        "target_speed_mph",
        "corner_speed_limit_mps",
        "v_mps",
        "a_mps2",
        "throttle",
        "brake",
        "energy_used_J",
        "energy_used_Wh",
        "power_W",
        "power_cmd_W",
        "power_wheel_W",
        "F_drive_N",
        "F_rr_N",
        "F_roll_N",
        "F_drag_N",
        "F_res_N",
        "F_brake_N",
        "F_net_N",
    ]
    keys = []
    seen = set()

    # add preferred keys that exist
    for k in preferred:
        if any(k in row for row in history) and k not in seen:
            keys.append(k)
            seen.add(k)

    # add everything else
    for row in history:
        for k in row.keys():
            if k not in seen:
                keys.append(k)
                seen.add(k)

    with p.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in history:
            writer.writerow(row)
