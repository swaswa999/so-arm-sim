"""Import the supplied Onshape STEP with names, colours, and assembly placement.
Inspection stage only: all parts fixed; mating and moving-part groups pending.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import cadquery as cq
import cadquery.occ_impl.importers.assembly as importer
from OCP.TDF import TDF_Tool
from OCP.TCollection import TCollection_AsciiString

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'assets/tire_tool'
# STEP contains repeated instance names. Append stable STEP label IDs for CQ.
original_name=importer._get_name
def unique_name(label):
    entry=TCollection_AsciiString()
    TDF_Tool.Entry_s(label,entry)
    return original_name(label)+'__'+entry.ToCString().replace(':','_')
importer._get_name=unique_name
assembly=cq.Assembly.load(str(OUT/'source.step'))
root=ET.Element('mujoco',model='Mayter custom wrist and socket — CAD inspection')
ET.SubElement(root,'compiler',angle='radian')
visual=ET.SubElement(root,'visual')
ET.SubElement(visual,'global',offwidth='1200',offheight='900',azimuth='145',elevation='-20')
ET.SubElement(visual,'headlight',ambient='0.4 0.4 0.4',diffuse='0.6 0.6 0.6')
asset=ET.SubElement(root,'asset')
world=ET.SubElement(root,'worldbody')
ET.SubElement(world,'light',pos='0 -0.3 0.5',dir='0 0 -1')
body=ET.SubElement(world,'body',name='custom_wrist_assembly')
parts=[]
for index,(shape,name,location,color) in enumerate(assembly):
    placed=shape.moved(location)
    filename=f'part_{index:02d}.stl'
    cq.exporters.export(placed,str(OUT/filename),tolerance=0.08,angularTolerance=0.15)
    bb=placed.BoundingBox()
    rgba=color.toTuple() if color else (0.7,0.7,0.7,1)
    part={'name':name,'mesh':filename,'rgba':rgba,'bounds_mm':[bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax]}
    parts.append(part)
    ET.SubElement(asset,'mesh',name=f'part_{index}',file='assets/tire_tool/'+filename,scale='.001 .001 .001')
    ET.SubElement(body,'geom',name=f'part_{index}',type='mesh',mesh=f'part_{index}',rgba=' '.join(map(str,rgba)),contype='0',conaffinity='0')
ET.indent(root)
ET.ElementTree(root).write(ROOT/'tire_inspection.xml',encoding='unicode')
(OUT/'parts.json').write_text(json.dumps({'source_url':'https://cad.onshape.com/documents/2ac14cfd423a4120f8cad5e6/w/acc8ed4486356bf30f43548c/e/912d94122f10ecaac60cf653','source_units':'mm','simulation_units':'m','stage':'geometry inspection; no verified joint groups, inertias, or docking transform','parts':parts},indent=2))
print('Exported',len(parts),'parts, preserved assembly placement and colours')
