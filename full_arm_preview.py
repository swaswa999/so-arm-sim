"""Full tool-change and wheel-service kinematic rehearsal. No hardware commands."""
from pathlib import Path
import argparse,time,math,json,copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from solve_arm import m as model,d as data,solve,ids,limits
ROOT=Path(__file__).resolve().parent
TIP=np.array([.012368,-.130,.012269])
layout=json.loads((ROOT/'assets/sim_two_tool_dock/design.json').read_text())
ROT=R.from_euler('x',180,degrees=True).as_matrix()@R.from_euler('y',-layout['housing_level_correction_deg'],degrees=True).as_matrix()
BASE_ROT=R.from_euler('x',180,degrees=True).as_matrix()
def orientation(target):return R.from_euler('z',-math.atan2(target[0],target[1]+.40)).as_matrix()@ROT
PARK={p['folder']:{'rotation':np.array(p['rotation']),'origin':np.array(p['translation_mm'])*.001} for p in layout['placements']}
for p in PARK.values():p['tip']=p['origin']+p['rotation']@TIP;p['axis']=p['rotation']@np.array([0,-1,0])
DOCKTIP=PARK['socket_tool']['tip'];DOCKROT=PARK['socket_tool']['rotation'];DOCKAXIS=PARK['socket_tool']['axis']
LIFTTIP=DOCKTIP+[0,0,.09];GROUND_TIP_Z=DOCKTIP[2]
SEATED=np.array([0,-.020,.054]);WHEEL_SEAT=np.array([0,-.013,.054])
NUT_REST=model.body('nut').pos.copy();WHEEL_REST=model.body('wheel').pos.copy()
model.geom_size[model.geom('table').id]=[.55,.60,.01]
state0=dict(target=DOCKTIP-.055*DOCKAXIS,jaw=-.3,attached='none',nut_mode='fixture',nut_out=0.,nut_angle=0.,wheel_mode='fixture',tire_open=.006)
phases=[];current=copy.deepcopy(state0)
def add(label,duration,**changes):
 global current
 end=copy.deepcopy(current);end.update(changes);phases.append((label,duration,copy.deepcopy(current),end));current=copy.deepcopy(end)
def event(**changes):current.update(changes)
def pickup(tool,first=False):
 p=PARK[tool];dock=p['tip'];axis=p['axis'];title='socket tool' if tool=='socket_tool' else 'tire tool';approach=dock-.055*axis
 if not first:
  add('Raise empty gripper',2,target=current['target']+[0,0,.09])
  add('Move above '+title+' dock',3,target=approach+[0,0,.09])
  add('Lower to '+title+' entry',3,target=approach)
 add('Slide gripper into '+title,3,target=dock)
 add('Lock '+title,2,jaw=0.)
 event(attached=tool)
 add('Lift '+title+' from dock',3,target=dock+[0,0,.09])
def putback(tool):
 p=PARK[tool];dock=p['tip'];axis=p['axis'];title='socket tool' if tool=='socket_tool' else 'tire tool'
 add('Lift clear of wheel fixture',2,target=current['target']+[0,0,.07])
 add('Return '+title+' above its dock',3,target=dock+[0,0,.09])
 add('Lower '+title+' into alignment guides',3,target=dock)
 add('Unlock '+title,2,jaw=-.3)
 event(attached='none')
 add('Withdraw gripper from '+title,3,target=dock-.055*axis)
add('Ready: both tools parked',1)
pickup('socket_tool',True)
add('Carry socket above nut',3,target=np.array([0,-.075,.12]))
add('Align socket with nut',2,target=np.array([0,-.075,.054]))
add('Seat socket on nut',3,target=SEATED)
add('Unscrew nut: eight turns',8,target=SEATED+[0,-.008,0],nut_out=.008,nut_angle=16*math.pi)
event(nut_mode='socket')
add('Withdraw socket with retained nut',3,target=SEATED+[0,-.045,0])
putback('socket_tool')
pickup('tire_tool')
add('Carry tire tool above wheel',3,target=np.array([0,-.085,.125]))
add('Align open jaws with wheel',3,target=np.array([0,-.085,.054]))
add('Slide jaws around wheel',4,target=WHEEL_SEAT)
add('Close tire jaws on wheel',2,tire_open=0.)
event(wheel_mode='tire')
add('Pull wheel completely off axle',4,target=WHEEL_SEAT+[0,-.070,0])
add('Lift removed wheel clear',3,target=WHEEL_SEAT+[0,-.070,.055])
add('Wheel removed: hold clear of axle',3)
add('Lower wheel to axle height',3,target=WHEEL_SEAT+[0,-.070,0])
add('Slide wheel back onto axle',4,target=WHEEL_SEAT)
event(wheel_mode='fixture')
add('Open tire jaws and release wheel',2,tire_open=.006)
add('Withdraw tire tool from wheel',4,target=WHEEL_SEAT+[0,-.075,0])
putback('tire_tool')
pickup('socket_tool')
add('Carry retained nut above axle',3,target=np.array([0,-.075,.12]))
add('Align nut with axle thread',3,target=np.array([0,-.075,.054]))
add('Bring nut onto axle end',4,target=SEATED+[0,-.008,0])
event(nut_mode='fixture',nut_out=.008)
add('Screw nut back on: eight turns',8,target=SEATED,nut_out=0.,nut_angle=0.)
add('Withdraw socket from installed nut',3,target=SEATED+[0,-.055,0])
putback('socket_tool')
add('Complete: wheel and nut installed, both tools parked',3)
PHASES=[(p[0],p[1]) for p in phases];TOTAL=sum(p[1] for p in phases);ENDS=np.cumsum([p[1] for p in phases])
def state(t):
 t=float(np.clip(t,0,TOTAL));i=min(int(np.searchsorted(ENDS,t,side='right')),len(phases)-1)
 label,duration,a,b=phases[i];start=0 if i==0 else ENDS[i-1];u=float(np.clip((t-start)/duration,0,1));u=u*u*(3-2*u)
 result={}
 for k in a:result[k]=a[k] if isinstance(a[k],str) else a[k]+(b[k]-a[k])*u
 result.update(label=label,index=i);return result
CACHE=ROOT/'full_cycle_trajectory.npz'
def plan():
 ts=np.linspace(0,TOTAL,int(TOTAL*18)+1);qs=[];errors=[];q=np.array([-1.57,.2,1.,-1.2,-1.52])
 for t in ts:
  s=state(t);q,e,a=solve(s['target'],q,orientation(s['target']));qs.append(q.copy());errors.append([e,a])
 errors=np.array(errors);worst=np.argmax(errors[:,0]);print('Max IK error mm/deg:',errors.max(0)*[1000,180/np.pi],state(ts[worst])['label'],flush=True)
 if errors[:,0].max()>.001 or errors[:,1].max()>.015:raise RuntimeError('Unreachable cycle; no viewer launched')
 np.savez(CACHE,t=ts,q=qs);return ts,np.array(qs)
def set_world_body(name,center,rotation):
 fixture=model.body('wheel_fixture').id
 model.body(name).pos[:]=center-data.xpos[fixture]
 q=R.from_matrix(rotation).as_quat();model.body(name).quat[:]=[q[3],*q[:3]]
def apply(t,ts,qs):
 s=state(t);data.qpos[ids]=[np.interp(t,ts,qs[:,k]) for k in range(5)]
 for name,value in [('locking_servo',s['jaw']),('nut_out',s['nut_out'] if s['nut_mode']=='fixture' else 0),('nut_turn',s['nut_angle']),('drive_turn',-s['nut_angle']),('nut_drop',0),('wheel_out',0),('tire_jaw_slide_15',s['tire_open']),('tire_jaw_slide_25',s['tire_open'])]:data.qpos[model.joint(name).qposadr[0]]=value
 model.body('nut').pos[:]=NUT_REST;model.body('nut').quat[:]=[1,0,0,0]
 model.body('wheel').pos[:]=WHEEL_REST;model.body('wheel').quat[:]=[1,0,0,0]
 mujoco.mj_forward(model,data)
 poses={}
 for name,p in PARK.items():
  mid=model.body(name).mocapid[0]
  if s['attached']==name:
   bid=model.body('custom_gripper').id;origin=data.xpos[bid].copy();rotation=data.xmat[bid].reshape(3,3).copy()
  else:origin=p['origin'];rotation=p['rotation']
  q=R.from_matrix(rotation).as_quat();data.mocap_pos[mid]=origin;data.mocap_quat[mid]=[q[3],*q[:3]];poses[name]=(origin,rotation)
 mujoco.mj_forward(model,data)
 if s['nut_mode']=='socket':
  origin,rotation=poses['socket_tool'];center=origin+rotation@TIP-(rotation@np.array([0,-1,0]))*.00165
  set_world_body('nut',center,rotation@BASE_ROT.T)
 if s['wheel_mode']=='tire':
  origin,rotation=poses['tire_tool'];set_world_body('wheel',origin+rotation@TIP,rotation@ROT.T)
 data.qvel[:]=0;mujoco.mj_forward(model,data)
 return s
# Cache exact rendered mesh vertices for floor tests.
mesh_geoms=[]
for g in range(model.ngeom):
 if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
  mi=model.geom_dataid[g];a=model.mesh_vertadr[mi];n=model.mesh_vertnum[mi];mesh_geoms.append((g,model.mesh_vert[a:a+n]))
def floor_clearance():return min(float((v@data.geom_xmat[g].reshape(3,3)[2]+data.geom_xpos[g,2]).min()) for g,v in mesh_geoms)
def camera(cam):cam.lookat[:]=[-.035,-.17,.105];cam.distance=.85;cam.azimuth=210;cam.elevation=-28

def validate(ts,qs):
 clearance=1.;error=0.;max_jump=0.;last=None
 for t in np.linspace(0,TOTAL,int(TOTAL*24)+1):
  s=apply(t,ts,qs);clearance=min(clearance,floor_clearance());error=max(error,float(np.linalg.norm(data.site_xpos[model.site('socket_axis_target').id]-s['target'])))
  assert np.isfinite(data.xpos).all() and (data.qpos[ids]>=limits[:,0]).all() and (data.qpos[ids]<=limits[:,1]).all()
  if s['wheel_mode']=='tire':assert s['attached']=='tire_tool' and s['tire_open']<1e-9
  points=np.vstack([data.xpos[model.body(n).id] for n in ['socket_tool','tire_tool','wheel','nut']])
  if last is not None:max_jump=max(max_jump,float(np.linalg.norm(points-last,axis=1).max()))
  last=points
 assert clearance>=-.0001,('floor',clearance)
 assert error<.001,('IK',error)
 assert max_jump<.01,('discontinuity',max_jump)
 final=apply(TOTAL,ts,qs);assert final['attached']=='none' and final['wheel_mode']=='fixture' and final['nut_mode']=='fixture' and abs(final['nut_out'])<1e-10
 assert np.allclose(model.body('wheel').pos,WHEEL_REST) and np.allclose(model.body('nut').pos,NUT_REST)
 for name,p in PARK.items():assert np.allclose(data.mocap_pos[model.body(name).mocapid[0]],p['origin'])
 report={'duration_seconds':TOTAL,'stages':len(phases),'sample_count':int(TOTAL*24)+1,'floor_clearance_mm':clearance*1000,'maximum_IK_error_mm':error*1000,'maximum_sample_body_displacement_mm':max_jump*1000,'joint_limits':'passed','end_state':'wheel and nut installed, both tools parked, gripper released','limitations':['kinematic animation, not force or contact dynamics','wheel grip and nut retention assumed','eight turns and 1 mm pitch retained from original printed-thread demo','full collision and physical hardware execution not validated']}
 (ROOT/'full_cycle_checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
def main():
 p=argparse.ArgumentParser();p.add_argument('--preview',action='store_true');p.add_argument('--video',action='store_true');p.add_argument('--replan',action='store_true');args=p.parse_args()
 if args.replan or not CACHE.exists():ts,qs=plan()
 else:z=np.load(CACHE);ts,qs=z['t'],z['q']
 if args.preview or args.video:
  validate(ts,qs)
  from PIL import Image,ImageDraw
  cam=mujoco.MjvCamera();camera(cam)
  with mujoco.Renderer(model,height=900,width=1200) as renderer:
   def frame(t):
    s=apply(t,ts,qs);renderer.update_scene(data,camera=cam);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,1200,68),fill='#17202c');dr.text((16,12),'MAYTER | '+s['label'],fill='white',font_size=21);dr.text((16,43),'Full wheel-service motion rehearsal | Tool retention and contact forces unvalidated',fill='#bcc7d7',font_size=15);return im
   for text,name in [('Ready:','start'),('Close tire jaws','grip-wheel'),('Wheel removed:','wheel-removed'),('Slide wheel back','wheel-replaced'),('Screw nut back','nut-installed'),('Complete:','complete')]:
    i=next(i for i,p in enumerate(phases) if p[0].startswith(text));t=ENDS[i]-.01;frame(t).save(ROOT/f'full-cycle-{name}.png')
   if args.video:
    import subprocess
    proc=subprocess.Popen(['ffmpeg','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1200x900','-r','20','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(ROOT/'full-wheel-service-cycle.mp4')],stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
    for t in np.arange(0,TOTAL,1/20):proc.stdin.write(np.asarray(frame(t)).tobytes())
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
   if controls['next']:t=next((float(x)+.001 for x in ENDS if x>t+.01),0.);controls['next']=False
   if controls['play']:t=min(t+dt*4,TOTAL)
   if t>=TOTAL:controls['play']=False
   with window.lock():s=apply(t,ts,qs)
   window.set_texts((100,0,'MAYTER | COMPLETE WHEEL SERVICE\n'+s['label']+'\n4x speed | P: play/pause  N: next stage  R: reset',f'{s["index"]+1}/{len(phases)} | {int(t)} / {int(TOTAL)} s\nKinematic rehearsal; grip and forces unvalidated'))
   window.sync();time.sleep(1/60)
if __name__=='__main__':main()
