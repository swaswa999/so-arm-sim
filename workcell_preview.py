"""Interactive CAD sequence rehearsal. Prescribed motion, not contact simulation."""
from pathlib import Path
import math
import time
import argparse
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
PITCH=.001  # metres per turn, from the supplied 3p4x1_CUSTOM part name
PHASES=[('Assembled',1.5),('Unscrew nut',4),('Place nut below wheel',2),('Remove wheel',3),('Wheel removed',1.5),('Replace wheel',3),('Bring nut to axle',2),('Reinstall nut',4)]
TOTAL=sum(d for _,d in PHASES)

def smooth(t):
    t=max(0,min(1,t));return t*t*(3-2*t)

def pose(t):
    t=t%TOTAL
    for i,(label,duration) in enumerate(PHASES):
        if t<duration: break
        t-=duration
    u=smooth(t/duration)
    out=drop=wheel=0.
    if i==1: out=.008*u
    elif i==2: out=.008;drop=.035*u
    elif i in (3,4,5):
        out=.008;drop=.035
        wheel=.06*(u if i==3 else 1 if i==4 else 1-u)
    elif i==6: out=.008;drop=.035*(1-u)
    elif i==7: out=.008*(1-u)
    return label,dict(nut_out=out,nut_turn=out/PITCH*2*math.pi,nut_drop=drop,wheel_out=wheel)

def apply(model,data,t):
    label,values=pose(t)
    for name,value in values.items(): data.qpos[model.joint(name).qposadr[0]]=value
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    return label

def camera_setup(camera):
    camera.lookat[:]=[.015,-.02,.035]
    camera.distance=.48;camera.azimuth=65;camera.elevation=-28

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(ROOT/'workcell.xml'));data=mujoco.MjData(model)
    if args.preview:
        # Check every frame for finite transforms and removal ordering.
        for t in np.linspace(0,TOTAL,1000):
            apply(model,data,t)
            assert np.isfinite(data.xpos).all()
            if data.qpos[model.joint('wheel_out').qposadr[0]]>0:
                assert data.qpos[model.joint('nut_drop').qposadr[0]]>=.0349
        from PIL import Image
        camera=mujoco.MjvCamera();camera_setup(camera)
        with mujoco.Renderer(model,height=900,width=1200) as renderer:
            for t,name in [(0,'workcell-assembled.png'),(11.8,'workcell-removed.png')]:
                apply(model,data,t);renderer.update_scene(data,camera=camera)
                Image.fromarray(renderer.render()).save(ROOT/name)
        print('29 CAD meshes loaded; 1000 sequence samples checked; previews saved.')
        return
    from mujoco import viewer
    state={'playing':False,'reset':False,'next':False}
    def key(code):
        if code==ord('P'): state['playing']=not state['playing']
        elif code==ord('R'): state['reset']=True
        elif code==ord('N'): state['next']=True
    t=0.;last=time.monotonic()
    with viewer.launch_passive(model,data,key_callback=key,show_left_ui=False,show_right_ui=False) as window:
        camera_setup(window.cam)
        while window.is_running():
            now=time.monotonic();dt=min(now-last,.1);last=now
            if state['reset']: t=0.;state['reset']=False;state['playing']=False
            if state['next']:
                boundaries=np.cumsum([d for _,d in PHASES])
                t=next((float(b)+.001 for b in boundaries if b>t%TOTAL+.01),0.)
                state['next']=False
            if state['playing']: t+=dt
            with window.lock():
                label=apply(model,data,t)
            window.set_texts((100,0,'MAYTER | REV 2 CAD REHEARSAL\n'+label+'\nP: play/pause   N: next stage   R: reset','Prescribed motion; contact forces not simulated\nCustom wrist shown separately pending mounting confirmation'))
            window.sync();time.sleep(1/60)
if __name__=='__main__':main()
