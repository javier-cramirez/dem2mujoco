from __future__ import annotations
from pathlib import Path
import time
import mujoco
import mujoco.viewer

def view_xml(xml_path: str | Path):
    xml_path = Path(xml_path).resolve()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Interactive orbit/zoom viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)
