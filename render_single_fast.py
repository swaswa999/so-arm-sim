"""Single clean full-cycle video, 4x playback; no hardware access."""
import subprocess
import numpy as np
import mujoco
from PIL import Image
import full_arm_preview as f
z=np.load(f.CACHE);ts,qs=z['t'],z['q']
W,H,FPS,SPEED=1200,900,24,4
out=f.ROOT/'Mayter-Single-Fast.mp4'
cam=mujoco.MjvCamera();f.camera(cam)
cmd=['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
p=subprocess.Popen(cmd,stdin=subprocess.PIPE)
times=np.minimum(np.arange(int((f.TOTAL/SPEED+1)*FPS))/FPS*SPEED,f.TOTAL)
with mujoco.Renderer(f.model,height=H,width=W) as r:
 for i,t in enumerate(times):
  f.apply(t,ts,qs);r.update_scene(f.data,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
  frame=r.render();p.stdin.write(frame.tobytes())
  if i==len(times)//2:Image.fromarray(frame).save(f.ROOT/'single-fast-preview.jpg')
  if i%200==0:print(f'{i}/{len(times)} frames',flush=True)
p.stdin.close();assert p.wait()==0
print(out,flush=True)
