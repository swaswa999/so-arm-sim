"""View the actual custom assembly before defining its joint mechanics."""
from pathlib import Path
import sys
import mujoco
ROOT=Path(__file__).resolve().parent
model=mujoco.MjModel.from_xml_path(str(ROOT/'socket_inspection.xml'))
data=mujoco.MjData(model)
mujoco.mj_forward(model,data)
if '--preview' in sys.argv:
    from PIL import Image
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[.012,-.025,.015]
    camera.distance=.36
    camera.azimuth=20
    camera.elevation=-20
    with mujoco.Renderer(model,height=900,width=1200) as renderer:
        renderer.update_scene(data,camera=camera)
        Image.fromarray(renderer.render()).save(ROOT/'socket-import.png')
    print('Compiled',model.nmesh,'CAD meshes; saved socket-import.png')
else:
    from mujoco import viewer
    print('Actual Onshape custom wrist/socket assembly — geometry inspection.')
    print('Joint grouping and attachment to the arm are not configured yet.')
    viewer.launch(model,data)
