"""Two open-front, three-sided funnel docks; concept fit pending physical test."""
from pathlib import Path
import json, math
import cadquery as cq
import numpy as np
import trimesh
from full_arm_preview import DOCKTIP,DOCKROT,TIP
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'assets/two_tool_guide';OUT.mkdir(exist_ok=True)
mesh=trimesh.load(ROOT/'assets/socket_tool/part_00.stl')
world=mesh.vertices*.001@DOCKROT.T+(DOCKTIP-DOCKROT@TIP)
u=DOCKROT[:2,0];v=np.array([-u[1],u[0]])
xy=world[:,:2]*1000@np.column_stack([u,v])
lo=xy.min(0);hi=xy.max(0);center=(lo+hi)/2
width,depth=hi-lo+1.2
height=18.;throat=3.;flare=10.;wall=3.;spacing=85.
# A straight 3 mm locating section below 15 mm high wedge ramps.
def rect(w,d,z):
 return cq.Workplane('XY',origin=(0,0,z)).rect(w,d).val()
outer=cq.Workplane('XY').box(width+2*(flare+wall),depth+2*(flare+wall),height,centered=(True,True,False))
inner=cq.Workplane('XY').box(width,depth,throat,centered=(True,True,False))
ramp=cq.Solid.makeLoft([rect(width,depth,throat),rect(width+2*flare,depth+2*flare,height)])
nest=outer.cut(inner).cut(ramp)
# Remove the entire fourth side; gripper approaches from negative local Y.
opening=cq.Workplane('XY').box(width+2*(flare+wall)+2,40,height+2,centered=(True,False,False)).translate((0,-depth/2-40,-1))
nest=nest.cut(opening)
angle=math.degrees(math.atan2(u[1],u[0]))
location=center[0]*u+center[1]*v+[-20,0]
def placed(shape,dx=0):return shape.rotate((0,0,0),(0,0,1),angle).translate((location[0]+dx,location[1],0))
first=placed(nest);second=placed(nest,-spacing)
# Low connecting web behind the funnel's closed side, clear of both cavities.
back=location+v*(depth/2+flare+wall/2)
web=cq.Workplane('XY').box(spacing,6,3,centered=(False,True,False)).translate((back[0]-spacing,back[1],0))
combined=first.union(second).union(web)
# A ground-level strut connects the right dock to the fixture datum.
start=location+u*(width/2+flare+wall/2)
end=np.array([-45.5,-31.]);delta=end-start;n=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)*4
strut=cq.Workplane('XY').polyline([p.tolist() for p in [start+n,start-n,end-n,end+n]]).close().extrude(3)
datum=cq.Workplane('XY').box(6,36,3,centered=(False,True,False)).translate((-48.5,-20,0))
combined=combined.union(strut).union(datum)
for yy in [-31,-9]:combined=combined.cut(cq.Workplane('XY').center(-45.5,yy).circle(1.7).extrude(4))
assert len(combined.solids().vals())==1, len(combined.solids().vals())
assert combined.val().isValid()
name='Mayter_Two_Tool_Funnel_Dock_Prototype'
cq.exporters.export(combined,str(OUT/(name+'.step')))
cq.exporters.export(combined,str(OUT/(name+'.stl')),tolerance=.04,angularTolerance=.1)
m=trimesh.load(OUT/(name+'.stl'));assert m.is_watertight and len(m.split())==1
(OUT/'design.json').write_text(json.dumps({'name':name,'units':'mm','height':height,'straight_locating_height':throat,'ramp_horizontal_flare':flare,'bay_spacing':spacing,'clearance_per_side':.6,'opening_width_depth':[width,depth],'size_mm':m.extents.tolist(),'bays':2,'second_bay':'duplicate socket housing envelope; second tool fit not established','status':'concept prototype; friction, gripper clearance, tool release, reach and drop-in alignment require validation','mounting':'register right datum against wheel fixture and fasten to common board','tool_support':'open bottom, tool rests on board'},indent=2))
print(json.loads((OUT/'design.json').read_text()))
