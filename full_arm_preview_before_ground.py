"""Full SO-101 joint-limited kinematic rehearsal with a 180-degree tool roll.
CAD wrist registration and contact/locking mechanics remain provisional.
"""
from pathlib import Path
import argparse,time,math,json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from solve_arm import m as model,d as data,solve,ids,limits
from engagement_preview import PHASES,TOTAL,ease,mix
ROOT=Path(__file__).resolve().parent
# Rotation X(pi) is the old Z(pi) orientation rolled 180 degrees about socket axis.
QUAT=np.array([0,1,0,0]);ROT=R.from_quat([1,0,0,0]).as_matrix()
TIP=np.array([.012368,-.130,.012269])
DOCKTIP=np.array([.10,-.065,.090]);LIFTTIP=DOCKTIP+[0,0,.015]
ABOVE=np.array([0,-.070,.105]);ALIGN=np.array([0,-.070,.054]);SEATED=np.array([0,-.020,.054])
def orientation(target):
 theta=math.atan2(target[0],target[1]+.40)
 return R.from_euler('z',-theta).as_matrix()@ROT
DOCKROT=orientation(DOCKTIP)
DOCKAXIS=DOCKROT@np.array([0,-1,0])
# Rest the inverted housing on the conceptual stand.
stand=model.geom('concept_tool_stand').id
stand_center=DOCKTIP-DOCKROT@TIP+DOCKROT@np.array([.013,-.074,.012269])
model.geom_pos[stand]=[stand_center[0],stand_center[1],.0255]
model.geom_size[model.geom('table').id]=[.55,.60,.01]
model.geom_size[stand]=[.032,.029,.0255]

def state(t):
 t=max(0,min(t,TOTAL-1e-8));elapsed=0
 for i,(label,duration) in enumerate(PHASES):
  if t<elapsed+duration:break
  elapsed+=duration
 u=ease((t-elapsed)/duration)
 tip=DOCKTIP.copy();grip_tip=DOCKTIP-.055*DOCKAXIS;jaw=-.3;out=angle=0.;locked=False
 if i==0:grip_tip=DOCKTIP+(-.055+.01*u)*DOCKAXIS
 elif i==1:grip_tip=DOCKTIP-.045*(1-u)*DOCKAXIS
 elif i==2:grip_tip=tip.copy();jaw=-.3*(1-u);locked=u>=.999
 else:
  locked=True;jaw=0.
  if i==3:tip=mix(DOCKTIP,LIFTTIP,u)
  elif i==4:tip=mix(LIFTTIP,ABOVE,u)
  elif i==5:tip=mix(ABOVE,ALIGN,u)
  elif i==6:tip=mix(ALIGN,SEATED,u)
  else:
   out=.008*(u if i==7 else 1);angle=out/.001*2*math.pi
   if i>=8:out+=.02*(u if i==8 else 1)
   tip=SEATED+[0,-out,0]
  grip_tip=tip.copy()
 return label,tip,grip_tip,jaw,out,angle,locked,i

CACHE=ROOT/'full_arm_trajectory.npz'
def plan():
 ts=np.linspace(0,TOTAL,621);qs=[];errors=[];q=np.array([-1.57,.2,1.,-1.2,-1.52])
 for t in ts:
  target=state(t)[2];q,e,a=solve(target,q,orientation(target));qs.append(q.copy());errors.append([e,a])
 errors=np.array(errors)
 if errors[:,0].max()>.001 or errors[:,1].max()>.015:raise RuntimeError(f'Unreachable trajectory: {errors.max(0)}')
 np.savez(CACHE,t=ts,q=qs)
 print('Planned 621 joint-limited poses. Max position error mm / angle deg:',errors[:,0].max()*1000,errors[:,1].max()*180/np.pi,flush=True)
 return ts,np.array(qs)

def apply(t,ts,qs):
 label,tip,target,jaw,out,angle,locked,i=state(t)
 data.qpos[ids]=[np.interp(t,ts,qs[:,k]) for k in range(5)]
 for joint,value in [('locking_servo',jaw),('nut_out',out),('nut_turn',angle),('drive_turn',-angle),('nut_drop',0),('wheel_out',0)]:data.qpos[model.joint(joint).qposadr[0]]=value
 mid=model.body('socket_tool').mocapid[0]
 rotation=orientation(target) if locked else DOCKROT
 data.mocap_pos[mid]=tip-rotation@TIP
 quat=R.from_matrix(rotation).as_quat();data.mocap_quat[mid]=[quat[3],*quat[:3]]
 mujoco.mj_forward(model,data)
 # Once locked, drive tool from actual arm FK rather than an independent animation.
 if locked:
  bid=model.body('custom_gripper').id
  data.mocap_pos[mid]=data.xpos[bid];data.mocap_quat[mid]=data.xquat[bid]
  mujoco.mj_forward(model,data)
 return label,locked

def floor_clearance():
 minimum=1.
 for gid in range(model.ngeom):
  mid=model.geom_dataid[gid]
  if model.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH:continue
  start=model.mesh_vertadr[mid];count=model.mesh_vertnum[mid]
  vertices=model.mesh_vert[start:start+count]
  minimum=min(minimum,float((vertices@data.geom_xmat[gid].reshape(3,3)[2]+data.geom_xpos[gid,2]).min()))
 return minimum

def camera(cam):
 cam.lookat[:]=[0,-.20,.12];cam.distance=.80;cam.azimuth=25;cam.elevation=-22

def main():
 p=argparse.ArgumentParser();p.add_argument('--preview',action='store_true');p.add_argument('--video',action='store_true');p.add_argument('--replan',action='store_true');a=p.parse_args()
 if a.replan or not CACHE.exists():ts,qs=plan()
 else:
  saved=np.load(CACHE);ts,qs=saved['t'],saved['q']
 if a.preview or a.video:
  clearance=1.;error=0.
  for t in np.linspace(0,TOTAL,1241):
   apply(t,ts,qs);clearance=min(clearance,floor_clearance())
   error=max(error,np.linalg.norm(data.site_xpos[model.site('socket_axis_target').id]-state(t)[2]))
   assert (data.qpos[ids]>=limits[:,0]).all() and (data.qpos[ids]<=limits[:,1]).all()
  assert clearance>=-.0001, f'Floor intersection: {clearance}'
  assert error<.001, f'IK interpolation error {error}'
  report={'samples':1241,'minimum_mesh_floor_clearance_mm':clearance*1000,'max_socket_target_error_mm':error*1000,'joint_limits':'passed','limitations':['prescribed joint motion, no force simulation','custom wrist axis registration inferred from CAD','self and fixture collision clearance not fully validated']}
  (ROOT/'full_arm_checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
  from PIL import Image,ImageDraw
  cam=mujoco.MjvCamera();camera(cam)
  with mujoco.Renderer(model,height=900,width=1200) as renderer:
   def frame(t):
    label,locked=apply(t,ts,qs);renderer.update_scene(data,camera=cam)
    im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,1200,64),fill='#17202c')
    dr.text((18,12),'MAYTER | Full arm | '+label,fill='white',font_size=22)
    dr.text((18,43),'Tool rolled 180 deg | Joint-limited kinematic rehearsal | Contact forces unvalidated',fill='#bcc7d7',font_size=15)
    return im
   for t,name in [(0,'start'),(7,'locked'),(22,'unscrew'),(31,'done')]:frame(t).save(ROOT/f'full-arm-{name}.png')
   if a.video:
    import subprocess
    proc=subprocess.Popen(['ffmpeg','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1200x900','-r','24','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(ROOT/'full-arm-engage-and-unscrew.mp4')],stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
    for t in np.arange(0,TOTAL,1/24):proc.stdin.write(np.asarray(frame(t)).tobytes())
    proc.stdin.close();assert proc.wait()==0
  return
 from mujoco import viewer
 controls={'play':False,'reset':False,'next':False};t=0.;last=time.monotonic()
 def key(code):
  if code==ord('P'):controls['play']=not controls['play']
  elif code==ord('R'):controls['reset']=True
  elif code==ord('N'):controls['next']=True
 with viewer.launch_passive(model,data,key_callback=key,show_left_ui=False,show_right_ui=False) as window:
  camera(window.cam)
  while window.is_running():
   now=time.monotonic();dt=min(now-last,.1);last=now
   if controls['reset']:t=0.;controls['reset']=False;controls['play']=False
   if controls['next']:t=next((float(x)+.001 for x in np.cumsum([d for _,d in PHASES]) if x>t+.01),0.);controls['next']=False
   if controls['play']:t=min(t+dt,TOTAL)
   if t>=TOTAL:controls['play']=False
   with window.lock():label,locked=apply(t,ts,qs)
   window.set_texts((100,0,'MAYTER | FULL ARM - TOOL FLIPPED 180\n'+label+'\nP: play/pause  N: next stage  R: reset','LOCKED' if locked else 'UNLOCKED'))
   window.sync();time.sleep(1/60)
if __name__=='__main__':main()
