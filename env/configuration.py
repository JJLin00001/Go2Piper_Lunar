"""Configuration shared by the builder and launcher (no Isaac imports)."""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENTS = ("Lunar", "Lunar_baseline")
BASELINE_CONFIG = ROOT / "env/config/Lunar_baseline.json"
DEFAULT_CONFIG = ROOT / "env/config/Lunar.json"
DEFAULT_SCENE = ROOT / "env/generated/Lunar.usda"
ROBOT_BUNDLE = ROOT / "assets/go2_piper_d435_assets"


def load_config(path: Path = DEFAULT_CONFIG) -> dict:
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_config(cfg)
    return cfg


def validate_config(cfg: dict) -> None:
    def positive(value, label, zero=False):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        if value < 0 or (not zero and value == 0):
            raise ValueError(f"{label} must be {'nonnegative' if zero else 'positive'}")

    if cfg.get("environment") not in ENVIRONMENTS:
        raise ValueError(f"environment must be one of {ENVIRONMENTS}")
    kind = cfg["terrain"].get("type")
    if kind not in ("procedural", "dem"):
        raise ValueError("terrain.type must be procedural or dem")
    if (cfg["environment"] == "Lunar") != (kind == "dem"):
        raise ValueError("Lunar requires DEM terrain; Lunar_baseline requires procedural terrain")
    positive(cfg["physics"]["gravity_m_s2"], "gravity")
    positive(cfg["physics"]["dt_s"], "physics dt")
    rate = 1 / cfg["physics"]["dt_s"]
    if not math.isclose(rate, round(rate), abs_tol=1e-8) or not 30 <= rate <= 2000:
        raise ValueError("dt_s must be 1 / integer Hz, with a rate from 30 to 2000 Hz")
    for key in ("solver_position_iterations", "solver_velocity_iterations"):
        v = cfg["physics"][key]
        if not isinstance(v, int) or v < 1:
            raise ValueError(f"{key} must be a positive integer")
    t, r, o = cfg["terrain"], cfg["robot"], cfg["ore"]
    for key in ("size_m", "spacing_m", "flat_radius_m", "flat_blend_m", "horizon_extent_m"):
        positive(t[key], key)
    positive(t["roughness_m"], "roughness", zero=True)
    if t["horizon_extent_m"] <= t["size_m"] / 2 + 1:
        raise ValueError("horizon_extent_m must exceed half the terrain size by more than 1 m")
    if t["spacing_m"] > t["flat_radius_m"] / 2:
        raise ValueError("terrain spacing is too coarse for the flat work area")
    if t["size_m"] / t["spacing_m"] > 1000:
        raise ValueError("terrain grid exceeds 1000 cells per side")
    if t["size_m"] / 2 < t["flat_radius_m"] + t["flat_blend_m"] + 2:
        raise ValueError("terrain must include space for craters outside the work area")
    if not math.isfinite(t["slope_deg"]) or abs(t["slope_deg"]) > 20:
        raise ValueError("slope_deg must be finite and within [-20, 20]")
    for key in ("crater_count", "rock_count"):
        if not isinstance(t[key], int) or t[key] < 0:
            raise ValueError(f"{key} must be a nonnegative integer")
    if kind == "dem":
        directory = Path(t["data_dir"])
        if directory.is_absolute() or ".." in directory.parts:
            raise ValueError("terrain.data_dir must be a repository-relative path")
        positive(t["detail_texture_period_m"], "detail texture period")
        if not isinstance(t["pebble_count"], int) or not 0 <= t["pebble_count"] <= 20000:
            raise ValueError("pebble_count must be an integer in [0, 20000]")
    positive(o["mass_kg"], "ore mass")
    if len(o["dimensions_m"]) != 3:
        raise ValueError("ore dimensions must have 3 entries")
    for v in o["dimensions_m"]:
        positive(v, "ore dimension")
    for group in (o,):
        for key in ("static_friction", "dynamic_friction"):
            positive(group[key], key, zero=True)
        if group["static_friction"] < group["dynamic_friction"]:
            raise ValueError("static friction must be >= dynamic friction")
    for prefix in ("foot", "pad", "body"):
        a, b = (r[f"{prefix}_{kind}_friction"] for kind in ("static", "dynamic"))
        positive(a, prefix + " static friction", zero=True)
        positive(b, prefix + " dynamic friction", zero=True)
        if a < b:
            raise ValueError("static friction must be >= dynamic friction")
    for group in (r, o):
        if len(group["xy_m"]) != 2 or not all(math.isfinite(v) for v in group["xy_m"]):
            raise ValueError("xy_m must have two finite entries")
        # Keep initial experiments in the controlled central patch.
        if math.hypot(*group["xy_m"]) + 0.6 > t["flat_radius_m"]:
            raise ValueError("robot/ore must initially lie inside the central work area")
    if len(r["arm_rad"]) != 6 or not all(math.isfinite(v) for v in r["arm_rad"]):
        raise ValueError("arm_rad must have six finite entries")
    if not 0 <= r["gripper_opening_m"] <= 0.1:
        raise ValueError("gripper_opening_m must be within [0, 0.1]")
    for key in ("hip_rad", "thigh_rad", "calf_rad", "yaw_deg"):
        if not math.isfinite(r[key]):
            raise ValueError(f"{key} must be finite")
    positive(r["foot_clearance_m"], "foot clearance", zero=True)
    positive(o["clearance_m"], "ore clearance", zero=True)
    for gains in r["pd"].values():
        if len(gains) != 2:
            raise ValueError("PD gains require [stiffness, damping]")
        for v in gains:
            positive(v, "PD gain", zero=True)
    if not 0 <= t["restitution"] <= 1:
        raise ValueError("restitution must be within [0, 1]")
    c = cfg["camera"]
    for key in ("width", "height", "sensor_width", "sensor_height"):
        if not isinstance(c[key], int) or c[key] < 16:
            raise ValueError(f"camera {key} must be an integer >= 16")
    if not 1 < c["sensor_horizontal_fov_deg"] < 179:
        raise ValueError("sensor FOV must be within (1, 179)")
    for key in ("overview_eye_m", "overview_target_m", "close_eye_m", "close_target_m"):
        if len(c[key]) != 3 or not all(math.isfinite(v) for v in c[key]):
            raise ValueError(f"{key} requires 3 finite coordinates")
    for key in ("sun_elevation_deg", "sun_azimuth_deg"):
        if not math.isfinite(cfg["lighting"][key]):
            raise ValueError(f"{key} must be finite")
    for prefix in ("overview", "close"):
        if c[f"{prefix}_eye_m"][:2] == c[f"{prefix}_target_m"][:2]:
            raise ValueError("View direction must not be parallel to the world Z axis")
    for key in ("sun_intensity", "fill_intensity"):
        positive(cfg["lighting"][key], key, zero=True)


def initial_joints(cfg: dict) -> dict[str, float]:
    r = cfg["robot"]
    q = {}
    for leg in ("FL", "FR", "RL", "RR"):
        q[f"{leg}_hip_joint"] = r["hip_rad"] * (1 if leg.endswith("L") else -1)
        q[f"{leg}_thigh_joint"] = r["thigh_rad"]
        q[f"{leg}_calf_joint"] = r["calf_rad"]
    q.update({f"joint{i + 1}": v for i, v in enumerate(r["arm_rad"])})
    q.update(joint7=r["gripper_opening_m"] / 2, joint8=-r["gripper_opening_m"] / 2)
    metadata = json.loads((ROBOT_BUNDLE / "joint_metadata.json").read_text())
    for j in metadata["joints"]:
        if not j["range"][0] <= q[j["name"]] <= j["range"][1]:
            raise ValueError(f"Initial position exceeds model limit: {j['name']}")
    return q


def apply_experiment_overrides(cfg, *, slope_deg=None, foot_static_friction=None,
                               foot_dynamic_friction=None, mass_kg=None):
    """Return a separate config; each experimental factor changes only itself."""
    import copy
    result = copy.deepcopy(cfg)
    for group, key, value in (
        ("terrain", "slope_deg", slope_deg),
        ("robot", "foot_static_friction", foot_static_friction),
        ("robot", "foot_dynamic_friction", foot_dynamic_friction),
        ("ore", "mass_kg", mass_kg),
    ):
        if value is not None:
            result[group][key] = value
    validate_config(result)
    return result


def foot_material_coefficients(cfg):
    """Ground uses unit multiply coefficients, so these are the effective pair values."""
    r = cfg["robot"]
    return r["foot_static_friction"], r["foot_dynamic_friction"]


def experiment_parameters(cfg):
    return {
        "slope_deg": cfg["terrain"]["slope_deg"],
        "slope_convention": "additive height tilt along projected/world +X; 0 preserves native DEM relief"
            if cfg["terrain"]["type"] == "dem" else "central planar patch angle along world +X",
        "foot_ground_static_friction": cfg["robot"]["foot_static_friction"],
        "foot_ground_dynamic_friction": cfg["robot"]["foot_dynamic_friction"],
        "ore_mass_kg": cfg["ore"]["mass_kg"],
    }
