"""Rev 2 dock: measured socket and tire housing pockets, both with three ramps."""
from pathlib import Path
import json,math
import numpy as np
import cadquery as cq
import trimesh
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'assets/sim_two_tool_dock';OUT.mkdir(exist_ok=True)
HEIGHT=18.;THROAT=3.;FLARE=10.;WALL=3.;CLEAR=.6
# Flatten the common housing face, rather than resting its CAD roll on an edge.
roll=math.degrees(math.atan2(.02604820799919571,.9996606878636524))
level=Rotation.from_euler('x',180,degrees=True).as_matrix()@Rotation.from_euler('y',-roll,degrees=True).as_matrix()
theta=math.degrees(math.atan2(.21558749,.97648453)); rot=Rotation.from_euler('z',theta,degrees=True).as_matrix()
u=rot[:2,0];v=rot[:2,1]
specs=[('socket_tool',0,[0,2,4,5,6,7,8,13,18,21]),('tire_tool',23,[0,3,4,5,6,7,8,9,10,12,14,15,16,17,18,19,21,22,23,24,25,27,28,31,32,33,35,38])]
bays=[]
def wire(w,d,z):return cq.Workplane('XY',origin=(0,0,z)).rect(w,d).val()
for folder,idx,ids in specs:
 flare=15. if folder=='tire_tool' else FLARE
 housing=trimesh.load(ROOT/f'assets/{folder}/part_{idx:02d}.stl');housing.vertices=housing.vertices@level.T
 floor=housing.bounds[0,2];housing.vertices[:,2]-=floor
 vs=housing.vertices;edges=vs[housing.edges_unique];cross=(edges[:,0,2]-THROAT)*(edges[:,1,2]-THROAT)<0
 e=edges[cross];fraction=(THROAT-e[:,0,2])/(e[:,1,2]-e[:,0,2]);hits=e[:,0]+fraction[:,None]*(e[:,1]-e[:,0])
 clipped=np.vstack([vs[vs[:,2]<=THROAT],hits]);bounds=np.array([clipped.min(0),clipped.max(0)]);center=(bounds[0,:2]+bounds[1,:2])/2
 width,depth=bounds[1,:2]-bounds[0,:2]+CLEAR*2
 # Enlarge upper funnel to guide toward the close-fitting bottom section.
 outer=cq.Workplane('XY').box(width+2*(flare+WALL),depth+2*(flare+WALL),HEIGHT,centered=(True,True,False))
 void=cq.Workplane('XY').box(width,depth,THROAT,centered=(True,True,False))
 taper=cq.Solid.makeLoft([wire(width,depth,THROAT),wire(width+2*flare,depth+2*flare,HEIGHT)])
 opening=cq.Workplane('XY').box(width+2*(flare+WALL)+2,100,HEIGHT+2,centered=(True,False,False)).translate((0,-depth/2-100,-1))
 nest=outer.cut(void).cut(taper).cut(opening)
 bays.append(dict(folder=folder,ids=ids,width=float(width),depth=float(depth),center=center,floor=float(floor),flare=flare,nest=nest))
# Simulation bays flank the fixture so both radial approaches stay within shoulder-pan limits.
bays[0]['location']=np.array([-92.,-69.]);bays[1]['location']=np.array([92.,-69.])
# Match the socket approach to the SO101's radial wrist orientation.
from scipy.optimize import brentq
offset=(level@np.array([12.368,-130.,12.269]))[:2]-bays[0]['center']
loc=bays[0]['location']
theta=math.degrees(brentq(lambda a:loc[0]*math.cos(a)+(loc[1]+400)*math.sin(a)+offset[0],0,math.pi/4))
rot=Rotation.from_euler('z',theta,degrees=True).as_matrix();u=rot[:2,0];v=rot[:2,1]
combined=None
backs=[]
placements=[]
for b in bays:
 loc=b['location']
 offset=(level@np.array([12.368,-130.,12.269]))[:2]-b['center']
 theta=math.degrees(brentq(lambda a:loc[0]*math.cos(a)+(loc[1]+400)*math.sin(a)+offset[0],-math.pi/3,math.pi/3))
 rot=Rotation.from_euler('z',theta,degrees=True).as_matrix();u=rot[:2,0];v=rot[:2,1]
 b['u']=u.copy()
 n=b['nest'].rotate((0,0,0),(0,0,1),theta).translate((*loc,0))
 combined=n if combined is None else combined.union(n)
 backs.append(loc+v*(b['depth']/2+b['flare']+WALL/2))
 trans=np.r_[-b['center'],-b['floor']]@rot.T+np.r_[loc,0]
 placements.append({'folder':b['folder'],'rotation':(rot@level).tolist(),'translation_mm':trans.tolist(),'tool_part_indices':b['ids']})
def strut(a,b,w=6):
 diff=b-a;perp=np.array([-diff[1],diff[0]])/np.linalg.norm(diff)*w/2
 return cq.Workplane('XY').polyline([p.tolist() for p in [a+perp,a-perp,b-perp,b+perp]]).close().extrude(3)
combined=combined.union(strut(backs[0],backs[1]))
start=bays[0]['location']+bays[0]['u']*(bays[0]['width']/2+FLARE+WALL/2)
combined=combined.union(strut(start,np.array([-45.5,-31.]),8))
combined=combined.union(cq.Workplane('XY').box(6,36,3,centered=(False,True,False)).translate((-48.5,-20,0)))
for y in [-31,-9]:combined=combined.cut(cq.Workplane('XY').center(-45.5,y).circle(1.7).extrude(4))
assert len(combined.solids().vals())==1 and combined.val().isValid()
name='Mayter_Socket_and_Tire_Funnel_Dock_Rev2'
cq.exporters.export(combined,str(OUT/(name+'.step')))
cq.exporters.export(combined,str(OUT/(name+'.stl')),tolerance=.04,angularTolerance=.1)
m=trimesh.load(OUT/(name+'.stl'));assert m.is_watertight and len(m.split())==1
report={'units':'mm','height':HEIGHT,'throat_height':THROAT,'flare':FLARE,'clearance_per_side':CLEAR,'bay_spacing':float(np.linalg.norm(bays[1]['location']-bays[0]['location'])),'size':m.extents.tolist(),'housing_level_correction_deg':roll,'bays':[{'tool':b['folder'],'opening_width':b['width'],'opening_depth':b['depth'],'flare':b['flare']} for b in bays],'placements':placements,'status':'CAD prototype, not physical drop-in or friction validation','limitations':['second bay reach and full robot path not validated','jaw motion envelope not supplied; checked supplied jaw pose only','one-sided open entry needs robot to seat tool against closed stop']}
(OUT/'design.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='placements'},indent=2))
