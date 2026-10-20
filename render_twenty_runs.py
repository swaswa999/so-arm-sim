"""Render twenty deterministic motion variations; no robot hardware access."""
import copy,json,subprocess,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import full_arm_preview as f
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'twenty_runs';OUT.mkdir(exist_ok=True)
BASE=copy.deepcopy(f.phases)
FPS=20;SPEED=2.;W,H=384,270;HEADER=84;LABEL=32
FONT='/System/Library/Fonts/Supplemental/Arial.ttf'
def font(n):return ImageFont.truetype(FONT,n)
variants=[]
for n in range(20):
 rng=np.random.default_rng(810+n);lift=float(rng.uniform(-.004,.006));reach=float(rng.uniform(-.004,.005))
 phases=copy.deepcopy(BASE)
 for i,(label,duration,a,b) in enumerate(phases):
  for s in (a,b):
   t=s['target'].copy();t[2]+=lift*np.clip((t[2]-.060)/.05,0,1)
   # Blend approach-distance changes away from precise contact and dock poses.
   t[1]-=reach*np.clip((-t[1]-.035)/.04,0,1)*np.clip(1-abs(t[0])/.04,0,1)
   s['target']=t
  phases[i]=(label,duration*float(rng.uniform(.90,1.10)),a,b)
 variants.append(dict(number=n+1,lift_mm=lift*1000,approach_mm=reach*1000,phases=phases,total=sum(p[1] for p in phases)))
def select(v):
 f.phases=v['phases'];f.ENDS=np.cumsum([p[1] for p in f.phases]);f.TOTAL=v['total']
def prepare(v):
 select(v);cache=OUT/f'run-{v["number"]:02}.npz'
 if cache.exists():
  z=np.load(cache);v['ts']=z['t'];v['qs']=z['q'];return
 ts=np.unique(np.r_[np.linspace(0,f.TOTAL,int(f.TOTAL*8)+1),0,f.ENDS]);qs=[];maxe=0;maxa=0;q=np.array([-1.57,.2,1,-1.2,-1.52])
 for t in ts:
  s=f.state(t);q,e,a=f.solve(s['target'],q,f.orientation(s['target']));qs.append(q.copy());maxe=max(maxe,e);maxa=max(maxa,a)
 assert maxe<.001 and maxa<.015,(v['number'],maxe,maxa)
 v['ts']=ts;v['qs']=np.array(qs)
 # Validate sampled motion, including exact phase boundaries and attachment events.
 floor=1.;error=0.
 for t in np.unique(np.r_[np.linspace(0,f.TOTAL,int(f.TOTAL*4)+1),f.ENDS,np.maximum(0,f.ENDS-.001)]):
  s=f.apply(t,ts,v['qs']);floor=min(floor,f.floor_clearance());error=max(error,float(np.linalg.norm(f.data.site_xpos[f.model.site('socket_axis_target').id]-s['target'])))
  assert np.isfinite(f.data.qpos).all()
  assert (f.data.qpos[f.ids]>=f.limits[:,0]).all() and (f.data.qpos[f.ids]<=f.limits[:,1]).all()
 assert floor>-.0001 and error<.001,(v['number'],floor,error)
 final=f.state(f.TOTAL);assert final['wheel_mode']=='fixture' and final['nut_mode']=='fixture' and final['attached']=='none' and final['nut_out']==0
 np.savez(cache,t=ts,q=v['qs']);v['checks']={'floor_clearance_mm':floor*1000,'tracking_error_mm':error*1000}
 print('Planned and checked run',v['number'],flush=True)
for v in variants:prepare(v)
report=[{k:v for k,v in item.items() if k not in ('phases','ts','qs')} for item in variants]
(OUT/'variations.json').write_text(json.dumps(report,indent=2))
length=max(v['total'] for v in variants)/SPEED+2;count=int(np.ceil(length*FPS));width=5*W;height=HEADER+4*(H+LABEL)
cam=f.mujoco.MjvCamera();f.camera(cam);cam.distance=.78;cam.lookat[:]=[-.015,-.17,.105]
output=ROOT/'Mayter-20-Parallel-Runs.mp4'
cmd=['ffmpeg','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
with open(OUT/'ffmpeg.log','w') as log:
 proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
 with f.mujoco.Renderer(f.model,height=H,width=W) as renderer:
  for frame in range(count):
   canvas=Image.new('RGB',(width,height),'#101823');draw=ImageDraw.Draw(canvas)
   draw.text((20,12),'MAYTER  /  20 COMPLETE WHEEL-SERVICE RUNS',font=font(27),fill='white')
   draw.text((20,50),'Small path + timing variations  |  2x playback  |  Kinematic animation; grip and contact forces unvalidated',font=font(19),fill='#a6b9ce')
   for n,v in enumerate(variants):
    select(v);s=f.apply(min(frame/FPS*SPEED,v['total']),v['ts'],v['qs'])
    renderer.update_scene(f.data,camera=cam);renderer.scene.flags[f.mujoco.mjtRndFlag.mjRND_SHADOW]=False;im=Image.fromarray(renderer.render());x=n%5*W;y=HEADER+n//5*(H+LABEL)
    canvas.paste(im,(x,y));draw.rectangle((x,y,x+W-1,y+H+LABEL-1),outline='#344459',width=1)
    label=s['label'];short=('COMPLETE' if label.startswith('Complete:') else label)
    if len(short)>37:short=short[:35]+'…'
    draw.text((x+9,y+H+7),f'{n+1:02}  {short}',font=font(15),fill='#83e7bf' if short=='COMPLETE' else '#dce7f2')
   proc.stdin.write(np.asarray(canvas).tobytes())
   if frame in (0,int(80/SPEED*FPS),count-1):canvas.save(OUT/f'contact-sheet-{frame:04}.jpg',quality=92)
   if frame%100==0:print(f'Rendered {frame}/{count} frames ({100*frame/count:.0f}%)',flush=True)
 proc.stdin.close();assert proc.wait()==0
print('DONE',output,flush=True)
