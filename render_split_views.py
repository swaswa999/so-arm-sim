"""Synchronized external and arm-side cameras. No hardware access."""
import subprocess
import numpy as np
import mujoco
from PIL import Image
import full_arm_preview as f
z=np.load(f.CACHE);ts,qs=z['t'],z['q']
W,H,FPS,SPEED=960,900,24,4
out=f.ROOT/'Mayter-Split-Perspective.mp4'
external=mujoco.MjvCamera();f.camera(external)
external.azimuth=235;external.elevation=-20;external.distance=.60;external.lookat[:]=[0,-.135,.10]
arm=mujoco.MjvCamera();arm.type=mujoco.mjtCamera.mjCAMERA_FREE
# Place the second camera above the robot base, looking toward the fixture.
# A fixed base view keeps the socket and tire visible during tool changes.
eye=np.array([-.0388353,-.45,.27]);target=np.array([0,-.04,.065]);delta=eye-target
arm.lookat[:]=target;arm.distance=np.linalg.norm(delta)
arm.azimuth=np.degrees(np.arctan2(delta[1],delta[0]));arm.elevation=-np.degrees(np.arcsin(delta[2]/arm.distance))
cmd=['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{2*W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
p=subprocess.Popen(cmd,stdin=subprocess.PIPE)
times=np.minimum(np.arange(int((f.TOTAL/SPEED+1)*FPS))/FPS*SPEED,f.TOTAL)
with mujoco.Renderer(f.model,height=H,width=W) as r:
 for i,t in enumerate(times):
  f.apply(t,ts,qs);views=[]
  bid=f.model.body('custom_gripper').id;origin=f.data.xpos[bid];rot=f.data.xmat[bid].reshape(3,3)
  eye=origin+rot@np.array([0,.045,-.10]);target=origin+rot@np.array([.012368,-.17,.012269]);delta=eye-target
  arm.lookat[:]=target;arm.distance=np.linalg.norm(delta)
  arm.azimuth=np.degrees(np.arctan2(delta[1],delta[0]))+180;arm.elevation=-np.degrees(np.arcsin(delta[2]/arm.distance))
  for cam in (external,arm):
   r.update_scene(f.data,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
   views.append(r.render().copy())
  frame=np.concatenate(views,axis=1);frame[:,W-1:W+1]=[35,45,55]
  p.stdin.write(frame.tobytes())
  if i==len(times)//2:Image.fromarray(frame).save(f.ROOT/'split-perspective-preview.jpg')
  if i%200==0:print(f'{i}/{len(times)} frames',flush=True)
p.stdin.close();assert p.wait()==0
print(out,flush=True)
