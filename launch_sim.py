"""Mayter simulation foundation. This program has no hardware connection."""
import argparse
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'assets/robotstudio_so101/scene.xml'

def load_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    home = np.array([0.0, -0.5, 0.7, 0.5, 0.0, 0.6])
    data.qpos[:6] = home
    data.ctrl[:6] = home
    mujoco.mj_forward(model, data)
    return model, data

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    model, data = load_model()
    if args.preview:
        for _ in range(1000):
            mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all():
            raise RuntimeError('Simulation produced invalid positions')
        model.vis.global_.offwidth = 960
        model.vis.global_.offheight = 720
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0.18]
        camera.distance = 0.85
        camera.azimuth = 140
        camera.elevation = -25
        from PIL import Image
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera)
            path = ROOT / 'robot-foundation.png'
            Image.fromarray(renderer.render()).save(path)
        print('Model compiled, 1000 physics steps completed, preview saved:', path)
        print('Joint positions:', data.qpos.tolist())
    else:
        from mujoco import viewer
        print('Mayter simulation foundation: robot only; waiting for tool and wheel CAD.')
        print('Use the Control panel to move simulated joints. No real motors are connected.')
        viewer.launch(model, data)

if __name__ == '__main__':
    main()
