from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
ROOT=Path(__file__).resolve().parent
m=mujoco.MjModel.from_xml_path(str(ROOT/'full_arm.xml'));d=mujoco.MjData(m)
names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
ids=[m.joint(n).qposadr[0] for n in names];limits=np.array([m.joint(n).range for n in names])
bid=m.body('custom_gripper').id;sid=m.site('socket_axis_target').id
ROT=R.from_euler('x',180,degrees=True).as_matrix()
def solve(target,start,orientation=None):
 target_rotation=ROT if orientation is None else orientation
 def residual(q):
  d.qpos[ids]=q;mujoco.mj_forward(m,d)
  rot=d.xmat[bid].reshape(3,3)
  return np.r_[d.site_xpos[sid]-target,.08*R.from_matrix(target_rotation@rot.T).as_rotvec()]
 r=least_squares(residual,start,bounds=(limits[:,0]+1e-5,limits[:,1]-1e-5),max_nfev=150)
 return r.x,np.linalg.norm(residual(r.x)[:3]),np.linalg.norm(residual(r.x)[3:])/.08
if __name__=='__main__':
 q=np.array([0,-.5,.7,.5,0])
 for target in [[0,-.12,.11],[0,-.065,.11],[0,-.065,.14],[0,-.07,.054],[0,-.02,.054]]:
  trials=[solve(target,q)]+[solve(target,np.random.default_rng(i).uniform(limits[:,0],limits[:,1])) for i in range(5)]
  q,err,ang=min(trials,key=lambda r:r[1]+.08*r[2]);print(target,np.round(q,3),err,ang)
