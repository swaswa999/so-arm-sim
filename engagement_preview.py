"""Prescribed CAD motion rehearsal; does not command hardware or solve contact forces."""
from pathlib import Path
import argparse, math, time, json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
PHASES=[('Approach tool in stand',2),('Slide gripper into tool',3),('Lock gripper servo',2),('Lift locked tool',2),('Carry tool to wheel',3),('Align socket with nut',2),('Seat socket on nut',3),('Unscrew nut - 1 mm per turn',8),('Withdraw with nut',3),('Nut removed',3)]
TOTAL=sum(d for _,d in PHASES)
DOCK=np.array([-.12,-.195,.07]);LIFT=DOCK+np.array([0,0,.055])
ABOVE=np.array([.012368,-.20,.125]);ALIGN=np.array([.012368,-.20,.054-.012269]);SEATED=ALIGN+np.array([0,.05,0])
def ease(x): return x*x*(3-2*x)
def mix(a,b,u): return a+(b-a)*u

def pose(t):
    t=max(0,min(t,TOTAL-1e-8));elapsed=0
    for i,(name,d) in enumerate(PHASES):
        if t<elapsed+d: break
        elapsed+=d
    u=ease((t-elapsed)/d)
    tool=DOCK.copy();grip=DOCK+np.array([0,-.075,0]);jaw=-.3;out=angle=0.;locked=False
    if i==0: grip=DOCK+np.array([0,-.075+.02*u,0])
    elif i==1: grip=DOCK+np.array([0,-.055*(1-u),0])
    elif i==2: grip=DOCK.copy();jaw=-.3*(1-u);locked=u>=.999
    else:
        locked=True;jaw=0.
        if i==3: tool=mix(DOCK,LIFT,u)
        elif i==4: tool=mix(LIFT,ABOVE,u)
        elif i==5: tool=mix(ABOVE,ALIGN,u)
        elif i==6: tool=mix(ALIGN,SEATED,u)
        else:
            out=.008*(u if i==7 else 1)
            angle=out/.001*2*math.pi
            if i>=8: out+=.02*(u if i==8 else 1)
            tool=SEATED+np.array([0,-out,0])
        grip=tool.copy()
    return name,tool,grip,jaw,out,angle,locked,i

def apply(model,data,t):
    name,tool,grip,jaw,out,angle,locked,i=pose(t)
    for body,pos in [('socket_tool',tool),('custom_gripper',grip)]:
        mid=model.body(body).mocapid[0];data.mocap_pos[mid]=pos;data.mocap_quat[mid]=[0,0,0,1]
    for joint,value in [('locking_servo',jaw),('nut_out',out),('nut_turn',angle),('drive_turn',-angle),('nut_drop',0),('wheel_out',0)]:
        data.qpos[model.joint(joint).qposadr[0]]=value
    data.qvel[:]=0;mujoco.mj_forward(model,data)
    return name,locked,i

def camera_setup(cam):
    cam.lookat[:]=[-.035,-.085,.065];cam.distance=.57;cam.azimuth=40;cam.elevation=-27

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');parser.add_argument('--video',action='store_true');args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(ROOT/'engagement.xml'));data=mujoco.MjData(model)
    if args.preview or args.video:
        # Verify continuity, lock-before-carry, axis alignment, and screw coupling.
        last=None
        for t in np.linspace(0,TOTAL,3301):
            name,tool,grip,jaw,out,angle,locked,i=pose(t)
            apply(model,data,t)
            assert np.isfinite(data.xpos).all()
            if i>=3: assert locked and np.linalg.norm(tool-grip)<1e-10
            if i==7:
                assert abs(out-angle*.001/(2*math.pi))<1e-10
                assert abs(tool[0]-.012368)<1e-10 and abs(tool[2]+.012269-.054)<1e-10
            if last is not None: assert np.linalg.norm(tool-last)<.003
            last=tool
        from PIL import Image,ImageDraw
        cam=mujoco.MjvCamera();camera_setup(cam)
        with mujoco.Renderer(model,height=900,width=1200) as renderer:
            for t,file in [(0,'engagement-start.png'),(6.9,'engagement-locked.png'),(22,'engagement-unscrew.png'),(32,'engagement-done.png')]:
                name,locked,i=apply(model,data,t);renderer.update_scene(data,camera=cam)
                im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,1200,65),fill='#17202c');draw.text((18,14),'MAYTER | '+name,fill='white',font_size=24)
                draw.text((18,46),'CAD motion rehearsal - fit, locking, and contact forces are unvalidated',fill='#bcc7d7',font_size=14)
                im.save(ROOT/file)
            if args.video:
                import subprocess
                cmd=['ffmpeg','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1200x900','-r','24','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(ROOT/'engage-and-unscrew.mp4')]
                proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
                for t in np.arange(0,TOTAL,1/24):
                    name,locked,i=apply(model,data,t);renderer.update_scene(data,camera=cam)
                    im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1200,65),fill='#17202c')
                    draw.text((18,12),'MAYTER | '+name,fill='white',font_size=24)
                    draw.text((18,44),'Motion rehearsal | Blue socket is a new concept adapter | No contact-force validation',fill='#bcc7d7',font_size=15)
                    proc.stdin.write(np.asarray(im).tobytes())
                proc.stdin.close();assert proc.wait()==0
        print('PASS: 3301 samples; continuous motion, docking before carry, socket alignment, screw pitch. Rendered previews.')
        return
    from mujoco import viewer
    state={'play':False,'reset':False,'next':False};t=0.;last=time.monotonic()
    def key(code):
        if code==ord('P'): state['play']=not state['play']
        elif code==ord('R'): state['reset']=True
        elif code==ord('N'): state['next']=True
    with viewer.launch_passive(model,data,key_callback=key,show_left_ui=False,show_right_ui=False) as window:
        camera_setup(window.cam)
        while window.is_running():
            now=time.monotonic();dt=min(now-last,.1);last=now
            if state['reset']:t=0.;state['reset']=False;state['play']=False
            if state['next']:
                t=next((float(x)+.001 for x in np.cumsum([d for _,d in PHASES]) if x>t+.01),0.);state['next']=False
            if state['play']:t=min(t+dt,TOTAL)
            if t>=TOTAL:state['play']=False
            with window.lock(): name,locked,i=apply(model,data,t)
            window.set_texts((100,0,'MAYTER | ENGAGE TOOL + UNSCREW\n'+name+'\nP: play/pause   N: next stage   R: reset',('LOCKED' if locked else 'UNLOCKED')+'\nPrescribed motion; fit and forces unvalidated\nBlue socket: newly generated concept adapter'))
            window.sync();time.sleep(1/60)
if __name__=='__main__':main()
