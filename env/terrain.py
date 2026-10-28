"""Terrain selection and audited, native-grid lunar DEM sampling (no GIS runtime)."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates

from configuration import ROOT
from geometry import LunarTerrain, vertex_normals


def file_sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


class DEMTerrain:
    def __init__(self, cfg):
        from PIL import Image
        self.cfg = cfg["terrain"]
        self.craters = []
        directory = ROOT / self.cfg["data_dir"]
        try:
            self.provenance = json.loads((directory / "provenance.json").read_text())
        except FileNotFoundError as error:
            raise FileNotFoundError(
                "Lunar DEM assets are missing. Run env/prepare_lunar_data.py with requirements-data.txt; "
                "there is no procedural fallback for Lunar.") from error
        for name in ("dem.npz", "ortho_reference.png"):
            if file_sha256(directory / name) != self.provenance["files"][name]:
                raise ValueError(f"DEM asset checksum mismatch: {directory / name}")
        with np.load(directory / "dem.npz", allow_pickle=False) as data:
            self.x = data["x_m"].astype(np.float64)
            self.y = data["y_m"].astype(np.float64)
            absolute = data["height_m"].astype(np.float64)
        if absolute.shape != (len(self.y), len(self.x)) or not np.isfinite(absolute).all():
            raise ValueError("Invalid DEM shape/elevations")
        self.dx, self.dy = np.diff(self.x)[0], np.diff(self.y)[0]
        if self.dx <= 0 or self.dy <= 0 or not np.allclose(np.diff(self.x), self.dx) or not np.allclose(np.diff(self.y), self.dy):
            raise ValueError("DEM coordinates must be uniformly spaced and ascending")
        self.elevation_origin = float(self.provenance["center_elevation_m"])
        self.elevations = absolute - self.elevation_origin
        extent = self.cfg["horizon_extent_m"]
        if self.x[0] > -extent or self.x[-1] < extent or self.y[0] > -extent or self.y[-1] < extent:
            raise ValueError("Configured terrain exceeds measured DEM coverage")
        self.ortho = gaussian_filter(np.asarray(Image.open(directory / "ortho_reference.png"), dtype=np.float64), 4.0)
        self.ortho_median = float(np.median(self.ortho))
        rng = np.random.default_rng(cfg["seed"] + 3000)
        self.detail_grid = gaussian_filter(rng.normal(size=(257, 257)), 1.2)
        self.detail_grid /= self.detail_grid.std()
        self.detail_center = float(self.detail_grid[128, 128])

    def native_height(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("Terrain query coordinates must be finite")
        if np.any((x < self.x[0]) | (x > self.x[-1]) | (y < self.y[0]) | (y > self.y[-1])):
            raise ValueError("Terrain query lies outside measured coverage")
        coordinates = np.vstack(((y.ravel() - self.y[0]) / self.dy, (x.ravel() - self.x[0]) / self.dx))
        return map_coordinates(self.elevations, coordinates, order=1, mode="nearest", prefilter=False).reshape(x.shape)

    def height(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        height = self.native_height(x, y)
        # Explicit synthetic millimetre detail; no flattening or invented craters.
        coordinates = np.vstack((y.ravel() * 4 + 128, x.ravel() * 4 + 128))
        detail = map_coordinates(self.detail_grid, coordinates, order=1, mode="nearest", prefilter=False).reshape(x.shape)
        fade = np.clip((30.0 - np.hypot(x, y)) / 10.0, 0, 1)
        fade = fade * fade * (3 - 2 * fade)
        height += self.cfg["roughness_m"] * (detail - self.detail_center) * fade
        return height + x * np.tan(np.deg2rad(self.cfg["slope_deg"]))

    def mesh(self):
        t = self.cfg
        half = t["size_m"] / 2
        near = np.linspace(-half, half, int(np.ceil(t["size_m"] / t["spacing_m"])) + 1)
        # More detail around the robot, sparse samples toward the measured horizon.
        far = np.geomspace(half + 0.5, t["horizon_extent_m"], 144)
        axis = np.concatenate((-far[::-1], near, far))
        xx, yy = np.meshgrid(axis, axis, indexing="xy")
        zz = self.height(xx, yy)
        vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())).astype(np.float32)
        n = len(axis)
        a = (np.arange(n - 1)[:, None] * n + np.arange(n - 1)[None, :]).ravel()
        faces = np.vstack((np.column_stack((a, a + 1, a + n + 1)),
                           np.column_stack((a, a + n + 1, a + n)))).astype(np.int32)
        left, bottom, right, top = self.provenance["ortho_reference_bounds_local_m"]
        rows = (top - yy.ravel()) / (top - bottom) * self.ortho.shape[0] - 0.5
        cols = (xx.ravel() - left) / (right - left) * self.ortho.shape[1] - 0.5
        tone = map_coordinates(self.ortho, [rows, cols], order=1, mode="nearest", prefilter=False)
        # Low-contrast image-derived tint only. Orbital brightness is not albedo.
        intensity = 0.18 * np.clip(tone / max(self.ortho_median, 1), 0.78, 1.22)
        colors = np.column_stack((intensity, intensity * 0.99, intensity * 0.97)).astype(np.float32)
        return vertices, faces, vertex_normals(vertices, faces), colors

    def report(self):
        x, y = np.meshgrid(np.linspace(-2, 2, 17), np.linspace(-2, 2, 17))
        matrix = np.column_stack((x.ravel(), y.ravel(), np.ones(x.size)))
        native_plane = np.linalg.lstsq(matrix, self.native_height(x, y).ravel(), rcond=None)[0]
        rendered_plane = np.linalg.lstsq(matrix, self.height(x, y).ravel(), rcond=None)[0]
        return {
            "type": "measured_dem_with_declared_synthetic_detail",
            "dataset": self.provenance["dataset"], "catalog_url": self.provenance["catalog_url"],
            "citation": self.provenance["citation"],
            "source_pixel_size_m": [self.dx, self.dy],
            "center_lon_lat_deg": self.provenance["center_lon_lat_deg"],
            "center_projected_m": self.provenance["center_projected_m"],
            "height_origin_above_reference_sphere_m": self.elevation_origin,
            "local_axes": "X projected easting, Y projected northing, Z elevation; units metres",
            "native_plane_gradient_xy": native_plane[:2].tolist(),
            "configured_plane_gradient_xy": rendered_plane[:2].tolist(),
            "configured_local_plane_slope_deg": float(np.rad2deg(np.arctan(np.linalg.norm(rendered_plane[:2])))),
            "source_files_sha256": self.provenance["files"],
            "synthetic_height_amplitude_parameter_m": self.cfg["roughness_m"],
            "synthetic_components": ["millimetre height detail", "regolith material detail", "scattered rocks and pebbles", "grasp target"],
            "image_usage": "Smoothed, contrast-limited orbital brightness modulates tint; not calibrated lunar albedo",
            "height_processing": "Bilinear DEM sampling, subtract center elevation, add declared synthetic detail and X tilt; no flattening",
        }


def make_terrain(cfg):
    return DEMTerrain(cfg) if cfg["terrain"]["type"] == "dem" else LunarTerrain(cfg)
