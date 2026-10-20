"""CAD-based engagement rehearsal with a newly designed socket adapter.
Prescribed kinematics only. Joint groups, jaw opening, and fit clearances are provisional.
"""
from pathlib import Path
import json, math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'assets/generated_socket';OUT.mkdir(exist_ok=True)
# Millimetres: 6.6 square recess for nominal 1/4-inch drive; 5.8 AF nut recess.
# Both ends open, connected by a transition. Axis matches the CAD drive axis.
N=96;cx,cz=12.368,12.269
angles=np.arange(N)*2*np.pi/N
# The supplied square drive is rotated relative to the assembly axes.
source=trimesh.load(ROOT/'assets/socket_tool/part_07.stl')
v=source.vertices[source.vertices[:,1]<-114.9][:,[0,2]]-np.array([cx,cz])
corner=v[np.argmax(np.linalg.norm(v,axis=1))]
square_angle=math.atan2(corner[1],corner[0])-np.pi/4

def polygon_radius(af,sides,rotation):
    d=(angles-rotation+np.pi/sides)%(2*np.pi/sides)-np.pi/sides
    return (af/2)/np.cos(d)
# Ring sequence outer rear/front, inner front/transition/drive/rear.
rings=[(-110,np.full(N,6.)),(-130,np.full(N,6.)),
       (-130,polygon_radius(5.8,6,0)),(-119,polygon_radius(5.8,6,0)),
       (-117,polygon_radius(6.6,4,square_angle)),(-110,polygon_radius(6.6,4,square_angle))]
verts=[]
for y,r in rings:
    verts.extend(np.column_stack([cx+r*np.cos(angles),np.full(N,y),cz+r*np.sin(angles)]))
faces=[]
for a,b in [(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)]:
    for j in range(N):
        k=(j+1)%N
        faces.extend([[a*N+j,b*N+j,b*N+k],[a*N+j,b*N+k,a*N+k]])
mesh=trimesh.Trimesh(verts,faces,process=True);mesh.fix_normals()
assert mesh.is_watertight
mesh.export(OUT/'socket_adapter.stl')
(OUT/'design.json').write_text(json.dumps({'units':'mm','length':20,'outer_diameter':12,'drive_recess_across_flats':6.6,'nut_recess_across_flats':5.8,'status':'concept adapter; clearance and insertion depth not physically validated','source':'generated for simulation, not supplied CAD'},indent=2))
root=ET.parse(ROOT/'workcell.xml').getroot()
root.set('model','Mayter — engage tool and unscrew nut')
world=root.find('worldbody');world.remove(world.find("body[@name='custom_tool_inspection']"))
asset=root.find('asset')
ET.SubElement(asset,'mesh',name='adapter_mesh',file='assets/generated_socket/socket_adapter.stl',scale='.001 .001 .001')
# Matching dock frames use the original Onshape assembled coordinates.
gripper=ET.SubElement(world,'body',name='custom_gripper',mocap='true',pos='-.12 -.25 .07',quat='0 0 0 1')
tool=ET.SubElement(world,'body',name='socket_tool',mocap='true',pos='-.12 -.195 .07',quat='0 0 0 1')
pivot=np.array([.012368,-.097607,.012269])
rotor=ET.SubElement(tool,'body',name='powered_drive',pos=' '.join(map(str,pivot)))
ET.SubElement(rotor,'joint',name='drive_turn',type='hinge',axis='0 -1 0',limited='false')
ET.SubElement(rotor,'inertial',pos='0 0 0',mass='.02',diaginertia='1e-6 1e-6 1e-6')
# CAD moving jaw pivot inferred from its opposed cylindrical bearing faces.
jawpivot=np.array([.01218,-.03018,-.00814])
jaw=ET.SubElement(gripper,'body',name='locking_jaw',pos=' '.join(map(str,jawpivot)))
ET.SubElement(jaw,'joint',name='locking_servo',type='hinge',axis='-0.99966 0 .02617',range='-.35 .1')
ET.SubElement(jaw,'inertial',pos='0 0 0',mass='.02',diaginertia='1e-6 1e-6 1e-6')
tool_indices={0,2,4,5,6,7,8,13,18,21};rotor_indices={2,7,13,21}
parts=json.loads((ROOT/'assets/socket_tool/parts.json').read_text())['parts']
for i,p in enumerate(parts):
    parent=rotor if i in rotor_indices else tool if i in tool_indices else jaw if i==14 else gripper
    offset=-pivot if parent is rotor else -jawpivot if parent is jaw else np.zeros(3)
    ET.SubElement(parent,'geom',name=f'engage_part_{i}',type='mesh',mesh=f'tool_mesh_{i}',pos=' '.join(map(str,offset)),rgba=' '.join(map(str,p['rgba'])),contype='0',conaffinity='0')
ET.SubElement(rotor,'geom',name='new_socket_adapter',type='mesh',mesh='adapter_mesh',pos=' '.join(map(str,-pivot)),rgba='.30 .55 .8 1',contype='0',conaffinity='0')
# Visual mock-up support only; actual rack CAD was not supplied.
ET.SubElement(world,'geom',name='concept_tool_stand',type='box',pos='-.132 -.12 .041',size='.032 .033 .041',rgba='.2 .23 .27 1',contype='0',conaffinity='0')
ET.indent(root);ET.ElementTree(root).write(ROOT/'engagement.xml',encoding='unicode')
print('Built engagement scene and watertight socket adapter')
