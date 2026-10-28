#!/usr/bin/env python3
"""Download georeferenced Apollo 17 windows, retaining source pixels and provenance.

Run with Python + requirements-data.txt. HTTP Range reads avoid downloading
the entire ~950 MB pair. No GIS dependencies are needed to run the saved scene.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CATALOG = "https://astrogeology.usgs.gov/search/map/moon_apollo_17_lroc_nac_landing_site_orthomosaic_50cm"
SOURCES = {
    "dem": "https://planetarymaps.usgs.gov/mosaic/Apollo17/APOLLO17_DTM_150CM.TIFF",
    "ortho": "https://asc-astropedia.s3.us-west-2.amazonaws.com/Moon/Apollo/Traverse/Apollo17/ancillary/APOLLO17_ORTHOMOSAIC_50CM.TIFF",
}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def prepare(output, lon, lat, extent):
    import requests
    import rasterio
    from rasterio.crs import CRS
    from rasterio.windows import Window
    from rasterio.warp import transform as project
    from PIL import Image

    output.mkdir(parents=True, exist_ok=True)
    previous = json.loads((output / "provenance.json").read_text()) if (output / "provenance.json").exists() else None
    metadata = {
        "dataset": "Apollo 17 LROC NAC DTM / orthomosaic (Haase et al.)",
        "catalog_url": CATALOG,
        "citation": "Haase et al. (2019), Coordinates and Maps of the Apollo 17 Landing Site, Earth and Space Science, doi:10.1029/2018EA000408",
        "doi": "https://doi.org/10.1029/2018EA000408",
        "producer": "Technical University Berlin / DLR / LROC Team; distributed by USGS",
        "retrieved_utc": datetime.now(timezone.utc).isoformat(),
        "requested_center_lon_lat_deg": [lon, lat],
        "reference_radius_m": 1737400.0,
        "height_datum": "Metres above lunar reference sphere of radius 1737400 m",
        "sources": {}, "files": {},
        "processing": [
            "Native-grid rectangular crops; no fabricated samples, no inpainting.",
            "DEM NPZ retains native float32 elevations and north-up georeferencing.",
            "Origin snaps to a DEM pixel center. Local X is projected east, Y is projected north.",
            "Orthomosaic PNG is reduced for visual reference; orbital shadows are not calibrated albedo.",
        ],
    }
    lunar_geo = CRS.from_string("+proj=longlat +a=1737400 +b=1737400 +no_defs")
    center = None
    with rasterio.Env(GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR",
                      CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".TIFF",
                      GDAL_HTTP_CONNECTTIMEOUT="30", GDAL_HTTP_TIMEOUT="180",
                      GDAL_HTTP_MAX_RETRY="3", VSI_CACHE=True,
                      CPL_VSIL_CURL_CACHE_SIZE=134217728):
        for kind, url in SOURCES.items():
            destination = output / (kind + "_crop.tif")
            headers = requests.head(url, timeout=30, allow_redirects=True)
            headers.raise_for_status()
            print(f"Reading {kind}: {url}", flush=True)
            with rasterio.open(url) as source:
                if center is None:
                    ex, ny = project(lunar_geo, source.crs, [lon], [lat])
                    row, col = source.index(ex[0], ny[0])
                    center = tuple(map(float, source.xy(row, col)))
                    snapped_lon, snapped_lat = project(source.crs, lunar_geo, [center[0]], [center[1]])
                    metadata["center_projected_m"] = list(center)
                    metadata["center_lon_lat_deg"] = [snapped_lon[0], snapped_lat[0]]
                    metadata["crs_wkt"] = source.crs.to_wkt()
                else:
                    if source.crs.to_wkt() != metadata["crs_wkt"]:
                        # Rasterio CRS equality is tolerant of WKT spelling differences.
                        if source.crs != CRS.from_wkt(metadata["crs_wkt"]):
                            raise ValueError("DEM/orthomosaic CRS mismatch; explicit reprojection required")
                    row, col = source.index(*center)
                half = int(np.ceil(extent / source.res[0]))
                window = Window(col - half, row - half, 2 * half + 1, 2 * half + 1)
                if window.col_off < 0 or window.row_off < 0 or window.col_off + window.width > source.width or window.row_off + window.height > source.height:
                    raise ValueError("Crop exceeds source coverage")
                crop_transform = source.window_transform(window)
                source_info = {
                    "url": url, "etag": headers.headers.get("ETag"),
                    "last_modified": headers.headers.get("Last-Modified"),
                    "remote_size_bytes": int(headers.headers.get("Content-Length", 0)),
                    "source_shape_rows_cols": [source.height, source.width],
                    "pixel_size_m": list(source.res),
                    "window_col_row_width_height": list(map(int, (window.col_off, window.row_off, window.width, window.height))),
                    "crop_transform": list(crop_transform)[:6],
                    "nodata": source.nodata,
                }
                if destination.exists():
                    if previous:
                        if sha256(destination) != previous["files"][destination.name]:
                            raise ValueError(f"Cached source crop failed checksum: {destination}")
                        if source_info["etag"] != previous["sources"][kind]["etag"]:
                            raise ValueError("Remote source version changed; choose a new output directory")
                    with rasterio.open(destination) as cached:
                        if cached.transform != crop_transform or cached.crs != source.crs or cached.shape != (int(window.height), int(window.width)):
                            raise ValueError(f"Existing crop differs from requested window: {destination}")
                        data = cached.read(1, masked=True)
                    print(f"Using existing crop {destination.name}", flush=True)
                else:
                    data = source.read(1, window=window, masked=True)
                    if np.ma.getmaskarray(data).any() or not np.isfinite(data).all():
                        raise ValueError(f"{kind}: crop contains missing/invalid source pixels; choose another crop")
                    profile = source.profile.copy()
                    profile.update(width=data.shape[1], height=data.shape[0],
                                   transform=crop_transform, compress="deflate", tiled=True,
                                   blockxsize=256, blockysize=256)
                    temporary = destination.with_suffix(".partial.tif")
                    with rasterio.open(temporary, "w", **profile) as dest:
                        dest.write(data.filled(), 1)
                        dest.update_tags(SOURCE_URL=url, CATALOG_URL=CATALOG)
                    temporary.replace(destination)
                if np.ma.getmaskarray(data).any() or not np.isfinite(data).all():
                    raise ValueError(f"Invalid cached {kind} data")
                values = np.asarray(data)
                source_info["crop_sha256"] = sha256(destination)
                metadata["sources"][kind] = source_info
                metadata["files"][destination.name] = source_info["crop_sha256"]
                if kind == "dem":
                    rows, cols = values.shape
                    x = crop_transform.c + (np.arange(cols) + 0.5) * crop_transform.a - center[0]
                    y = crop_transform.f + (np.arange(rows) + 0.5) * crop_transform.e - center[1]
                    np.savez_compressed(output / "dem.npz", x_m=x, y_m=y[::-1],
                                        height_m=values[::-1].copy())
                    metadata["center_elevation_m"] = float(values[half, half])
                    metadata["dem_range_m"] = [float(values.min()), float(values.max())]
                else:
                    if values.dtype != np.uint8:
                        raise ValueError("Expected source orthomosaic to be uint8; do not silently rescale")
                    im = Image.fromarray(values)
                    im.thumbnail((2048, 2048), Image.Resampling.LANCZOS)
                    im.save(output / "ortho_reference.png")
                    metadata["ortho_reference_bounds_local_m"] = [
                        crop_transform.c - center[0],
                        crop_transform.f + values.shape[0] * crop_transform.e - center[1],
                        crop_transform.c + values.shape[1] * crop_transform.a - center[0],
                        crop_transform.f - center[1],
                    ]
                print(f"{kind}: {values.shape}, range [{values.min()}, {values.max()}]", flush=True)
    for name in ("dem.npz", "ortho_reference.png"):
        metadata["files"][name] = sha256(output / name)
    metadata["coverage_half_extent_m"] = extent
    (output / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("DATA_READY", output, flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "assets/terrain_data/apollo17")
    p.add_argument("--longitude", type=float, default=30.7717)
    p.add_argument("--latitude", type=float, default=20.1908)
    p.add_argument("--half-extent-m", type=float, default=1536.0)
    args = p.parse_args()
    if args.half_extent_m <= 20:
        p.error("half extent must exceed 20 m")
    prepare(args.output.resolve(), args.longitude, args.latitude, args.half_extent_m)
