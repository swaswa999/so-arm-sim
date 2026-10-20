from pathlib import Path
import os
import json,xml.etree.ElementTree as ET
import mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'assets/two_tool_guide_rev2'
r=json.loads((OUT/'design.json').read_text())
root=ET.Element('mujoco');vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth='1400',offheight='1000');ET.SubElement(vis,'headlight',ambient='.5 .5 .5',diffuse='.5 .5 .5')
asset=ET.SubElement(root,'asset');w=ET.SubElement(root,'worldbody');ET.SubElement(w,'light',pos='0 -.2 1');ET.SubElement(w,'geom',type='plane',size='1 1 .01',rgba='.16 .19 .23 1')
ET.SubElement(asset,'mesh',name='dock',file='Mayter_Socket_and_Tire_Funnel_Dock_Rev2.stl',scale='.001 .001 .001');ET.SubElement(w,'geom',type='mesh',mesh='dock',rgba='.22 .65 .45 1')
for p in r['placements']:
 folder=p['folder'];parts=json.loads((ROOT/'assets'/folder/'parts.json').read_text())['parts'];q=Rotation.from_matrix(p['rotation']).as_quat();quat=[q[3],*q[:3]]
 body=ET.SubElement(w,'body',pos=' '.join(str(x*.001) for x in p['translation_mm']),quat=' '.join(map(str,quat)))
 for i in p['tool_part_indices']:
  part=parts[i];name=folder+str(i);ET.SubElement(asset,'mesh',name=name,file=os.path.relpath(ROOT/'assets'/folder/part['mesh'],OUT),scale='.001 .001 .001');ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,rgba=' '.join(map(str,part['rgba'])))
xml=ET.tostring(root,encoding='unicode');(OUT/'inspection.xml').write_text(xml)
m=mujoco.MjModel.from_xml_path(str(OUT/'inspection.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
c=mujoco.MjvCamera();c.lookat[:]=[-.14,-.065,.025];c.distance=.38;c.azimuth=110;c.elevation=-42
with mujoco.Renderer(m,1000,1400) as renderer:
 for label in ['with-tools','empty']:
  if label=='empty':
   for i in range(2,m.ngeom):m.geom_rgba[i,3]=0
  renderer.update_scene(d,camera=c);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1400,55),fill='#17202c');draw.text((24,20),'REV 2 | Tire-tool bay + socket-tool bay | 18 mm three-sided funnel guides',fill='white');im.save(OUT/(label+'.png'))
