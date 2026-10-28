"""Reproducible synthetic sub-pixel material detail, separate from measured DEM."""
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter
from PIL import Image


def create_textures(directory, seed, period_m):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed + 4100)
    n = 1024
    white = rng.normal(size=(n, n))
    fields = []
    for sigma in (0.7, 2.5, 9, 32):
        field = gaussian_filter(white, sigma, mode="wrap")
        fields.append(field / field.std())
    grain = 0.45 * fields[0] + 0.3 * fields[1] + 0.15 * fields[2] + 0.1 * fields[3]
    color = np.clip(0.82 + 0.065 * grain, 0.45, 0.99)
    # Periodic finite differences produce seamless, tangent-space normal detail.
    height = 0.00035 * fields[0] + 0.0006 * fields[1] + 0.001 * fields[2]
    dx = (np.roll(height, -1, 1) - np.roll(height, 1, 1)) / (2 * period_m / n)
    dy = (np.roll(height, -1, 0) - np.roll(height, 1, 0)) / (2 * period_m / n)
    # Image rows run downward, whereas USD texture V runs upward.
    normal = np.stack((-dx, dy, np.ones_like(dx)), axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    roughness = np.clip(0.92 + 0.025 * fields[1], 0.78, 0.99)
    arrays = {"color": color, "normal": normal * 0.5 + 0.5, "roughness": roughness}
    paths = {}
    for name, values in arrays.items():
        path = directory / (name + ".png")
        Image.fromarray(np.round(values * 255).astype(np.uint8)).save(path)
        paths[name] = path
    return paths
