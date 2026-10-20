from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
ROOT=Path(__file__).resolve().parent
scene=E.parse(ROOT/'engagement.xml').getroot();world=scene.find('worldbody')
stand=world.find("geom[@name='concept_tool_stand']")
if stand is not None:world.remove(stand)
custom=world.find("body[@name='custom_gripper']");world.remove(custom)
custom.attrib.pop('mocap');custom.attrib.pop('pos');custom.attrib.pop('quat')
robot=E.parse(ROOT/'assets/robotstudio_so101/so101.xml').getroot()
for mesh in robot.findall('./asset/mesh'):
 mesh.set('file',str(Path('assets/robotstudio_so101/assets')/mesh.get('file')))
scene.find('asset').extend(list(robot.find('asset')))
scene.append(robot.find('default'))
base=robot.find('./worldbody/body');world.append(base);base.set('pos','-.0388353 -.40 .0025')
wrist=base.find('.//body[@name="wrist"]');old=wrist.find("body[@name='gripper']")
roll=E.Element('body',name='gripper',pos=old.get('pos'),quat=old.get('quat'))
roll.append(old.find("joint[@name='wrist_roll']"));roll.append(old.find('inertial'))
for child in list(wrist):
 if child.tag in ['geom','body']:wrist.remove(child)
wrist.append(roll)
# Register the custom CAD's wrist-roll axis to the robot's wrist-roll frame.
A=R.from_euler('x',90,degrees=True).as_matrix();pivot=np.array([.012509,-.006024,.012008])
q=R.from_matrix(A).as_quat();custom.set('quat',' '.join(map(str,[q[3],*q[:3]])))
custom.set('pos',' '.join(map(str,-A@pivot)));roll.append(custom)
# Rear fork and roll motor remain on the wrist-flex link, ahead of the roll joint.
rear=E.SubElement(wrist,'body',name='custom_rear_wrist',pos=old.get('pos'),quat=old.get('quat'))
rearframe=E.SubElement(rear,'body',name='custom_rear_cad',pos=custom.get('pos'),quat=custom.get('quat'))
for i in [3,12,15,16,20]:
 g=custom.find(f"geom[@name='engage_part_{i}']")
 if g is not None:custom.remove(g);rearframe.append(g)
E.SubElement(custom,'site',name='socket_axis_target',pos='.012368 -.130 .012269',size='.001',rgba='0 0 0 0')
# The animation positions joints directly; disable collision proxies for display.
for g in base.iter('geom'):
 if g.get('class','').startswith('collision'):g.set('rgba','0 0 0 0')
 g.set('contype','0');g.set('conaffinity','0')
guide=ROOT/'assets/sim_two_tool_dock/Mayter_Socket_and_Tire_Funnel_Dock_Rev2.stl'
if guide.exists():
 E.SubElement(scene.find('asset'),'mesh',name='docking_guide_mesh',file=str(guide.relative_to(ROOT)),scale='.001 .001 .001')
 E.SubElement(world,'geom',name='floor_docking_guide',type='mesh',mesh='docking_guide_mesh',rgba='.15 .70 .55 1',contype='0',conaffinity='0')
# Park the real tire-tool CAD in the second fitted bay (no duplicate wrist).
layout=json.loads((ROOT/'assets/sim_two_tool_dock/design.json').read_text())
tire=next(p for p in layout['placements'] if p['folder']=='tire_tool')
q=R.from_matrix(tire['rotation']).as_quat()
parked=E.SubElement(world,'body',name='tire_tool',mocap='true',pos=' '.join(str(x*.001) for x in tire['translation_mm']),quat=' '.join(map(str,[q[3],*q[:3]])))
parts=json.loads((ROOT/'assets/tire_tool/parts.json').read_text())['parts']
for i in tire['tool_part_indices']:
 part=parts[i];name=f'tire_tool_part_{i}'
 E.SubElement(scene.find('asset'),'mesh',name=name,file=str(Path('assets/tire_tool')/part['mesh']),scale='.001 .001 .001')
 parent=parked
 if i in [15,25]:
  parent=E.SubElement(parked,'body',name=f'tire_jaw_{i}')
  E.SubElement(parent,'joint',name=f'tire_jaw_slide_{i}',type='slide',axis=('.99966 0 -.02605' if i==15 else '-.99966 0 .02605'),range='0 .008')
  E.SubElement(parent,'inertial',pos='0 0 0',mass='.01',diaginertia='1e-6 1e-6 1e-6')
 E.SubElement(parent,'geom',name=name,type='mesh',mesh=name,rgba=' '.join(map(str,part['rgba'])),contype='0',conaffinity='0')
E.indent(scene);E.ElementTree(scene).write(ROOT/'full_arm.xml',encoding='unicode')
print('Full arm with custom wrist/gripper registered at wrist-roll axis')
