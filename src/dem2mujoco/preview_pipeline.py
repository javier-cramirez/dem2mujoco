from __future__ import annotations
import os
import time
from pathlib import Path
import mujoco
import mujoco.viewer

from dem2mujoco.heightfield import dem_to_heightfield
from dem2mujoco.mjcf import write_terrain_xml

def preview_dem(
    dem_file_path: str | Path,
    workdir: str | Path = "dem2mujoco_out",
    *,
    target_n: int = 257,
    smooth_sigma: float = 1.0,
    z_scale_m: float = 3.0,
    x_half: float = 20.0,
    y_half: float = 20.0,
    base: float = 0.1,
    friction: tuple[float, float, float] = (1.0, 0.005, 0.0001),
):  
    dem_path = Path(dem_file_path).expanduser().resolve()
    workdir = Path(workdir).expanduser().resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    out_png_path = workdir / "terrain.png"
    out_xml_path = workdir / "terrain.xml"

    dem_to_heightfield(
        in_tif_path=str(dem_path),
        out_png_path=str(out_png_path),
        target_n=target_n,
        smooth_sigma=smooth_sigma,
        z_scale_m=z_scale_m,
    )

    write_terrain_xml(
        out_xml_path=str(out_xml_path),
        heightfield_png=out_png_path.name,
        x_half=x_half,
        y_half=y_half,
        z_scale=z_scale_m,
        base=base,
        friction=friction,
    )

    cwd = Path.cwd()
    try:
        os.chdir(workdir)
        model = mujoco.MjModel.from_xml_path(str(out_xml_path.name))
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.001)
    finally:
        os.chdir(cwd)

    return out_xml
