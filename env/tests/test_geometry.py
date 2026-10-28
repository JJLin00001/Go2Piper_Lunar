"""Numerical contracts behind terrain contact and payload inertia."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial import ConvexHull

from configuration import BASELINE_CONFIG, load_config, validate_config, initial_joints
from geometry import LunarTerrain, rock_mesh, mass_properties


class SceneGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(BASELINE_CONFIG)

    def test_work_area_is_flat_and_slopes_match_requested_angle(self):
        terrain = LunarTerrain(self.cfg)
        x = np.linspace(-0.9, 0.9, 19)
        xx, yy = np.meshgrid(x, x)
        np.testing.assert_array_equal(terrain.height(xx, yy), np.zeros_like(xx))
        self.cfg["terrain"]["slope_deg"] = 8
        terrain = LunarTerrain(self.cfg)
        np.testing.assert_allclose(terrain.height(xx, yy), xx * np.tan(np.deg2rad(8)), atol=1e-14)

    def test_terrain_is_repeatable_and_mesh_faces_point_up(self):
        # Test a smaller grid while preserving the same generation path.
        self.cfg["terrain"]["spacing_m"] = 0.6
        a, b = LunarTerrain(self.cfg), LunarTerrain(self.cfg)
        vertices, faces, normals, _ = a.mesh()
        np.testing.assert_array_equal(vertices, b.mesh()[0])
        np.testing.assert_allclose(vertices[:, 2], a.height(vertices[:, 0], vertices[:, 1]), atol=2e-6)
        normal = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                          vertices[faces[:, 2]] - vertices[faces[:, 0]])
        self.assertTrue(np.all(normal[:, 2] > 0))
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-6)
        changed = copy.deepcopy(self.cfg)
        changed["seed"] += 1
        self.assertFalse(np.array_equal(vertices, LunarTerrain(changed).mesh()[0]))

    def test_ore_is_closed_and_has_expected_dimensions(self):
        vertices, faces = rock_mesh(44, [0.18, 0.065, 0.15])
        np.testing.assert_allclose(np.ptp(vertices, axis=0), [0.18, 0.065, 0.15], atol=1e-8)
        edges = {}
        for a, b, c in faces:
            for i, j in ((a, b), (b, c), (c, a)):
                edges.setdefault(tuple(sorted((int(i), int(j)))), []).append((int(i), int(j)))
        self.assertTrue(all(len(v) == 2 and v[0] == v[1][::-1] for v in edges.values()))

    def test_integrated_inertia_matches_known_box_and_mass_scaling(self):
        dims = np.array([2.0, 3.0, 4.0])
        vertices = np.array([[x, y, z] for x in [-1, 1] for y in [-1.5, 1.5] for z in [-2, 2]])
        hull = ConvexHull(vertices)
        faces = hull.simplices.copy()
        for i, (tri, equation) in enumerate(zip(faces, hull.equations)):
            a, b, c = vertices[tri]
            if np.dot(np.cross(b - a, c - a), equation[:3]) < 0:
                faces[i] = tri[[0, 2, 1]]
        volume, center, inertia, axes = mass_properties(vertices, faces, 5.0)
        self.assertAlmostEqual(volume, 24)
        np.testing.assert_allclose(center, 0, atol=1e-14)
        expected = 5 / 12 * (np.sum(dims**2) - dims**2)
        np.testing.assert_allclose(axes @ np.diag(inertia) @ axes.T, np.diag(expected), atol=1e-12)
        _, _, heavier, _ = mass_properties(vertices, faces, 15.0)
        np.testing.assert_allclose(heavier, 3 * inertia)
        shifted = vertices + [7, -2, 8]
        _, shifted_center, shifted_inertia, _ = mass_properties(shifted, faces, 5)
        np.testing.assert_allclose(shifted_center, [7, -2, 8], atol=1e-12)
        np.testing.assert_allclose(shifted_inertia, inertia, atol=1e-10)

    def test_invalid_physics_and_joint_limits_are_rejected(self):
        validate_config(self.cfg)
        self.assertEqual(len(initial_joints(self.cfg)), 20)
        for bad in (-1, 0, float("nan"), float("inf")):
            cfg = copy.deepcopy(self.cfg)
            cfg["ore"]["mass_kg"] = bad
            with self.assertRaises(ValueError):
                validate_config(cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg["robot"]["arm_rad"][1] = -0.1
        with self.assertRaises(ValueError):
            initial_joints(cfg)


if __name__ == "__main__":
    unittest.main()
