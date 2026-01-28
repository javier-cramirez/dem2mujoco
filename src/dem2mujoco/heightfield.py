from __future__ import annotations

from pathlib import Path
import numpy as np
import rasterio
from scipy.ndimage import zoom, gaussian_filter
import imageio.v2 as imageio

def dem_to_heightfield(
    in_tif_path: str | Path,
    out_png_path: str | path,
    *,
    target_n: int = 257,
    smooth_sigma: float = 1.0,
    z_scale_m: float = 5.0 
):
    """
    Converts a DEM GeoTIFF file to a compatible 16-bit PNG heightfield (for Mujoco).
    """
    in_tif_path = Path(in_tif_path)
    out_png_path = Path(out_png_path)
    out_png_path.parent.mkdir(parents=True, exist_ok=True)

    with rasterio.open(in_tif_path) as src:
        elevation = src.read(1).astype(np.float32)
        nodata = src.nodata

    # we begin by dealing with nodata values (mask -> fill)
    if nodata is None:
        nodata = -9999.0
    mask = (elevation == nodata) | ~np.isfinite(elevation)
    elevation = np.where(mask, np.nan, elevation)

    if np.isnan(elevation).all():
        raise ValueError("All DEM values are NaN after performing nodata masking.")

    fill_value = float(np.nanmedian(elevation))
    elevation = np.where(np.isnan(elevation), fill_value, elevation)

    # center-crop to square -> resample
    h, w = elevation.shape
    side = min(h, w)
    r_0 = (h - side) // 2
    c_0 = (w - side) // 2
    elevation_square = elevation[r_0: r_0 + side, c_0: c_0 + side]

    scale = target_n / side
    # spline interpolation of order n 
    elevation_resampled = zoom(elevation_square, zoom=scale, order=1)

    # increases smoothness of the multidimensional array, if present
    if smooth_sigma and smooth_sigma > 0:
        elevation_resampled = gaussian_filter(elevation_resampled, sigma=smooth_sigma)

    # normalization
    elevation_resampled -= elevation_resampled.min()
    range_ = elevation_resampled.max() - elevation_resampled.min()
    if range_ < 1e-8:
        raise ValueError("Terrain has near-zero height range after processing.")
    elevation_norm = elevation_resampled / range_

    # save to 16-bit png
    image_16 = (elevation_norm * 65535).astype(np.uint16)
    imageio.imwrite(out_png_path.as_posix(), image_16)

    return out_png_path.as_posix()


