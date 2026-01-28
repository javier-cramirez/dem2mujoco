from pathlib import Path

MJCF_TEMPLATE = """\
<mujoco model="terrain_scene">
  <compiler angle="degree" coordinate="local"/>
  <option timestep="0.002"/>

  <asset>
    <hfield name="terrain"
            file="{hfield_png}"
            size="{x_half} {y_half} {z_scale} {base}"/>
  </asset>

  <worldbody>
    <geom type="hfield"
          hfield="terrain"
          pos="0 0 0"
          friction="{friction}"
          rgba="0.6 0.5 0.4 1"/>
  </worldbody>
</mujoco>
"""

def write_terrain_xml(
    out_xml_path: str | Path,
    heightfield_png: str,
    x_half: float,
    y_half: float,
    z_scale: float,
    base: float = 0.1,
    friction: tuple[float, float, float] = (1.0, 0.005, 0.0001)
):

    xml = MJCF_TEMPLATE.format(
        hfield_png = heightfield_png,
        x_half = x_half,
        y_half = y_half,
        z_scale = z_scale,
        base = base,
        friction = " ".join(f"{x:.6g}" for x in friction)
    )
    
    out_xml_path = Path(out_xml_path)
    out_xml_path.write_text(xml)
    return out_xml_path
