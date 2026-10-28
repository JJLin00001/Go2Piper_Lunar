"""Data orientation, real-source fidelity and independent experiment controls."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from configuration import ROOT, load_config, apply_experiment_overrides
from terrain import make_terrain, file_sha256
from launch import parse_args


class EnvironmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()
        cls.terrain = make_terrain(cls.cfg)

    def test_native_dem_matches_downloaded_geotiff_and_georeferencing(self):
        directory = ROOT / self.cfg["terrain"]["data_dir"]
        raw = np.asarray(Image.open(directory / "dem_crop.tif"), dtype=np.float64)
        with np.load(directory / "dem.npz") as data:
            np.testing.assert_array_equal(data["height_m"], raw[::-1])
            np.testing.assert_array_equal(data["x_m"], self.terrain.x)
            np.testing.assert_array_equal(data["y_m"], self.terrain.y)
        provenance = json.loads((directory / "provenance.json").read_text())
        a, b, c, d, e, f = provenance["sources"]["dem"]["crop_transform"]
        ox, oy = provenance["center_projected_m"]
        np.testing.assert_allclose(self.terrain.x[[0, -1]], c + (np.array([0, raw.shape[1] - 1]) + 0.5) * a - ox)
        np.testing.assert_allclose(self.terrain.y[[-1, 0]], f + (np.array([0, raw.shape[0] - 1]) + 0.5) * e - oy)
        for name, checksum in provenance["files"].items():
            self.assertEqual(file_sha256(directory / name), checksum)
        indices = np.array([10, 800, 1024, 1700, 2000])
        z = self.terrain.native_height(self.terrain.x[indices], self.terrain.y[indices])
        np.testing.assert_allclose(z, raw[::-1][indices, indices] - provenance["center_elevation_m"], atol=1e-10)

    def test_dem_keeps_native_relief_and_rejects_out_of_coverage(self):
        self.assertGreater(np.ptp(self.terrain.native_height([0, 0, 0], [-1.5, 0, 1.5])), 0.01)
        with self.assertRaises(ValueError):
            self.terrain.native_height(5000, 0)
        with self.assertRaises(ValueError):
            self.terrain.native_height(float("nan"), 0)

    def test_slope_is_additive_with_unchanged_dem_and_seed(self):
        cfg = apply_experiment_overrides(self.cfg, slope_deg=7)
        changed = make_terrain(cfg)
        x, y = np.meshgrid(np.linspace(-4, 4, 31), np.linspace(-4, 4, 31))
        np.testing.assert_array_equal(changed.native_height(x, y), self.terrain.native_height(x, y))
        np.testing.assert_allclose(changed.height(x, y) - self.terrain.height(x, y),
                                   x * np.tan(np.deg2rad(7)), atol=1e-12)
        self.assertEqual(cfg["robot"], self.cfg["robot"])
        self.assertEqual(cfg["ore"], self.cfg["ore"])

    def test_mass_and_foot_friction_do_not_change_terrain_or_gripper_contact(self):
        changed = apply_experiment_overrides(self.cfg, mass_kg=5,
                                             foot_static_friction=0.6, foot_dynamic_friction=0.5)
        self.assertEqual(changed["terrain"], self.cfg["terrain"])
        self.assertEqual(changed["ore"]["dimensions_m"], self.cfg["ore"]["dimensions_m"])
        self.assertEqual(changed["ore"]["static_friction"], self.cfg["ore"]["static_friction"])
        self.assertEqual(changed["robot"]["pad_static_friction"], self.cfg["robot"]["pad_static_friction"])
        self.assertEqual(self.cfg["ore"]["mass_kg"], 2.0)
        points = np.linspace(-2, 2, 13)
        np.testing.assert_array_equal(make_terrain(changed).height(points, points),
                                      self.terrain.height(points, points))

    def test_cli_names_and_variant_paths_do_not_overwrite_defaults(self):
        baseline, bc = parse_args(["--env", "Lunar_baseline"])
        lunar, lc = parse_args([])
        variant, vc = parse_args(["--env", "Lunar", "--mass-kg", "5"])
        self.assertEqual(baseline.scene.name, "Lunar_baseline.usda")
        self.assertEqual(lunar.scene.name, "Lunar.usda")
        self.assertNotEqual(variant.scene, lunar.scene)
        self.assertEqual(bc["terrain"]["type"], "procedural")
        self.assertEqual(lc["terrain"]["type"], "dem")
        self.assertEqual(vc["ore"]["mass_kg"], 5.0)
        again, _ = parse_args(["--env", "Lunar", "--mass-kg", "5"])
        self.assertEqual(again.scene, variant.scene)

    def test_invalid_experiments_fail_before_starting_isaac(self):
        for parameters in ({"mass_kg": 0}, {"slope_deg": float("nan")}, {"slope_deg": 30},
                           {"foot_static_friction": 0.1}, {"foot_dynamic_friction": -1}):
            with self.assertRaises(ValueError):
                apply_experiment_overrides(self.cfg, **parameters)


if __name__ == "__main__":
    unittest.main()
