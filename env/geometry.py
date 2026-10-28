"""Deterministic mesh geometry, independent of Isaac Sim and USD."""
from __future__ import annotations

import numpy as np
from scipy.spatial import ConvexHull


class LunarTerrain:
    def __init__(self, cfg: dict):
        self.cfg = cfg["terrain"]
        rng = np.random.default_rng(cfg["seed"])
        self.waves = [(rng.uniform(0.2, 1.6), rng.uniform(0.2, 1.6),
                       rng.uniform(0, 2 * np.pi)) for _ in range(6)]
        self.craters = []
        t = self.cfg
        for _ in range(t["crater_count"]):
            for _attempt in range(1000):
                xy = rng.uniform(-0.44 * t["size_m"], 0.44 * t["size_m"], 2)
                radius = rng.uniform(0.4, 2.0)
                if np.linalg.norm(xy) > t["flat_radius_m"] + t["flat_blend_m"] + radius:
                    self.craters.append((*xy, radius, rng.uniform(0.14, 0.24) * radius))
                    break
            else:
                raise ValueError("Could not place craters outside the work area")

    def height(self, x, y):
        x, y = np.asarray(x), np.asarray(y)
        z = np.zeros(np.broadcast_shapes(x.shape, y.shape))
        for kx, ky, phase in self.waves:
            z += self.cfg["roughness_m"] / 3 * np.sin(kx * x + phase) * np.cos(ky * y - phase)
        for cx, cy, radius, depth in self.craters:
            rho = np.hypot(x - cx, y - cy) / radius
            basin = -depth * np.maximum(0, 1 - rho**2)**2
            rim = 0.28 * depth * np.exp(-((rho - 1.03) / 0.16)**2)
            z += basin + rim
        blend = np.clip((np.hypot(x, y) - self.cfg["flat_radius_m"]) / self.cfg["flat_blend_m"], 0, 1)
        blend = blend * blend * (3 - 2 * blend)
        return z * blend + x * np.tan(np.deg2rad(self.cfg["slope_deg"]))

    def mesh(self):
        t = self.cfg
        half = t["size_m"] / 2
        near = np.linspace(-half, half, int(np.ceil(t["size_m"] / t["spacing_m"])) + 1)
        far = np.geomspace(half + 1, t["horizon_extent_m"], 48)
        axis = np.concatenate((-far[::-1], near, far))
        xx, yy = np.meshgrid(axis, axis, indexing="xy")
        zz = self.height(xx, yy)
        vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())).astype(np.float32)
        n = len(axis)
        a = (np.arange(n - 1)[:, None] * n + np.arange(n - 1)[None, :]).ravel()
        faces = np.vstack((np.column_stack((a, a + 1, a + n + 1)),
                           np.column_stack((a, a + n + 1, a + n)))).astype(np.int32)
        normals = vertex_normals(vertices, faces)
        # Neutral regolith variation; no network textures and no invented DEM provenance.
        variation = (0.022 * np.sin(0.47 * xx) * np.cos(0.73 * yy)
                     + 0.010 * np.sin(17 * xx + 13 * yy)
                     + 0.006 * np.cos(31 * xx - 23 * yy))
        intensity = np.clip(0.19 + variation.ravel(), 0.10, 0.28)
        colors = np.column_stack((intensity, intensity * 0.98, intensity * 0.94)).astype(np.float32)
        return vertices, faces, normals, colors


def vertex_normals(vertices, faces):
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    face_normals = np.cross(b - a, c - a)
    normals = np.zeros_like(vertices)
    for i in range(3):
        np.add.at(normals, faces[:, i], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    return normals.astype(np.float32)


def rock_mesh(seed: int, dimensions, count=46):
    rng = np.random.default_rng(seed)
    # Fibonacci directions avoid duplicate poles and form a closed convex rock.
    index = np.arange(count)
    z = 1 - 2 * (index + 0.5) / count
    phi = index * (np.pi * (3 - np.sqrt(5)))
    radius = np.sqrt(1 - z * z)
    vertices = np.column_stack((radius * np.cos(phi), radius * np.sin(phi), z))
    vertices *= rng.uniform(0.80, 1.15, (count, 1))
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    for i, (tri, equation) in enumerate(zip(faces, hull.equations)):
        a, b, c = vertices[tri]
        if np.dot(np.cross(b - a, c - a), equation[:3]) < 0:
            faces[i] = tri[[0, 2, 1]]
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    vertices = (vertices - (low + high) / 2) / (high - low) * np.asarray(dimensions)
    return vertices.astype(np.float32), faces.astype(np.int32)


def mass_properties(vertices, faces, mass):
    """Exact uniform-density mass properties of a closed, oriented convex mesh."""
    a, b, c = (vertices.astype(np.float64)[faces[:, i]] for i in range(3))
    volumes = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    volume = volumes.sum()
    if volume <= 0:
        raise ValueError("Rock mesh must be closed with outward normals")
    center = np.sum(volumes[:, None] * (a + b + c) / 4, axis=0) / volume
    s = a + b + c
    second = sum(np.einsum("ni,nj->nij", p, p) for p in (s, a, b, c))
    second = np.sum(volumes[:, None, None] * second / 20, axis=0) * mass / volume
    inertia = np.trace(second) * np.eye(3) - second
    inertia -= mass * (np.dot(center, center) * np.eye(3) - np.outer(center, center))
    moments, axes = np.linalg.eigh(inertia)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    if np.min(moments) <= 0:
        raise ValueError("Rock inertia must be positive definite")
    return float(volume), center, moments, axes
