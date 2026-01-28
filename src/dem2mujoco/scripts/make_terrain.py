from pathlib import Path
from dem2mujoco.heightfield import dem_to_heightfield
from dem2mujoco.mjcf import write_terrain_xml

out = Path("/tmp/dem2mujoco_test")
out.mkdir(parents=True, exist_ok=True)

png = out / 'terrain.png'
xml = out / 'terrain.xml'

dem_to_heightfield('tests/NorthMountain1.tin.tif', png, target_n=257, z_scale_m=3.0)
write_terrain_xml(xml, 'terrain.png', x_half=20, y_half=20, z_scale=3.0)

