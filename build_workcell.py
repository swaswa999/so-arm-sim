"""Build a geometry/kinematics preview; contact mechanics are not validated."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent
root=ET.Element('mujoco',model='Mayter workcell — geometry and sequence preview')
ET.SubElement(root,'compiler',angle='radian',autolimits='true')
ET.SubElement(root,'option',gravity='0 0 0',timestep='.005')
vis=ET.SubElement(root,'visual')
ET.SubElement(vis,'global',offwidth='1200',offheight='900',azimuth='-90',elevation='-25')
ET.SubElement(vis,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6')
asset=ET.SubElement(root,'asset')
world=ET.SubElement(root,'worldbody')
ET.SubElement(world,'light',pos='0 -.2 .6',dir='0 0 -1')
ET.SubElement(world,'geom',name='table',type='plane',size='.45 .35 .01',rgba='.13 .17 .22 1',contype='0',conaffinity='0')

parts=json.loads((ROOT/'assets/wheel_rev2/parts.json').read_text())['parts']
fixture=ET.SubElement(world,'body',name='wheel_fixture',pos='0 0 0.002')
for i,p in enumerate(parts):
    ET.SubElement(asset,'mesh',name=f'wheel_mesh_{i}',file='assets/wheel_rev2/'+p['mesh'],scale='.001 .001 .001')
    parent=fixture
    name=p['name']
    if '_P05_' in name:
        parent=ET.SubElement(fixture,'body',name='nut',pos='0 -.02165 .052')
        ET.SubElement(parent,'inertial',pos='0 0 0',mass='.002',diaginertia='1e-7 1e-7 1e-7')
        ET.SubElement(parent,'joint',name='nut_out',type='slide',axis='0 -1 0',range='0 .035')
        ET.SubElement(parent,'joint',name='nut_drop',type='slide',axis='0 0 -1',range='0 .035')
        ET.SubElement(parent,'joint',name='nut_turn',type='hinge',axis='0 -1 0',limited='false')
        offset='0 .02165 -.052'
    elif '_P04_' in name:
        parent=ET.SubElement(fixture,'body',name='wheel',pos='0 -.013 .052')
        ET.SubElement(parent,'inertial',pos='0 0 0',mass='.01',diaginertia='1e-6 1e-6 1e-6')
        ET.SubElement(parent,'joint',name='wheel_out',type='slide',axis='0 -1 0',range='0 .07')
        offset='0 .013 -.052'
    else: offset='0 0 0'
    ET.SubElement(parent,'geom',name=f'wheel_part_{i}',type='mesh',mesh=f'wheel_mesh_{i}',pos=offset,rgba=' '.join(map(str,p['rgba'])),contype='0',conaffinity='0')
# Display the supplied custom assembly next to fixture while mounting is confirmed.
tool=ET.SubElement(world,'body',name='custom_tool_inspection',pos='-.10 .04 .06')
for i,p in enumerate(json.loads((ROOT/'assets/socket_tool/parts.json').read_text())['parts']):
    ET.SubElement(asset,'mesh',name=f'tool_mesh_{i}',file='assets/socket_tool/'+p['mesh'],scale='.001 .001 .001')
    ET.SubElement(tool,'geom',name=f'tool_part_{i}',type='mesh',mesh=f'tool_mesh_{i}',rgba=' '.join(map(str,p['rgba'])),contype='0',conaffinity='0')
ET.indent(root)
ET.ElementTree(root).write(ROOT/'workcell.xml',encoding='unicode')
print('Built workcell: actual wheel fixture + custom tool, 29 CAD meshes')
