from pathlib import Path
import json
import cadquery as cq
import cadquery.occ_impl.importers.assembly as importer
from OCP.TDF import TDF_Tool
from OCP.TCollection import TCollection_AsciiString
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'assets/wheel_reprints'
original_name=importer._get_name
def unique_name(label):
    entry=TCollection_AsciiString();TDF_Tool.Entry_s(label,entry)
    return original_name(label)+'__'+entry.ToCString().replace(':','_')
importer._get_name=unique_name
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
reader=STEPCAFControl_Reader()
reader.SetNameMode(True);reader.SetColorMode(True)
reader.ReadFile(str(OUT/'rev2_exploded_source.step'))
doc=TDocStd_Document(TCollection_ExtendedString("XCAF"));reader.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
labels=TDF_LabelSequence();st.GetFreeShapes(labels)
assembly=[]
for label in labels:
    shape=st.GetShape_s(label)
    assembly.append((cq.Shape.cast(shape),original_name(label),cq.Location(),importer._get_shape_color(shape,ct)))
parts=[]
for index,(shape,name,location,color) in enumerate(assembly):
    placed=shape.moved(location)
    filename=f'part_{index:02d}.stl'
    cq.exporters.export(placed,str(OUT/filename),tolerance=0.03,angularTolerance=0.1)
    bb=placed.BoundingBox()
    part={'name':name,'mesh':filename,'rgba':color.toTuple() if color else (.6,.6,.6,1),'bounds_mm':[bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax]}
    parts.append(part)
    print(index,name,[round(x,3) for x in part['bounds_mm']],flush=True)
    cq.exporters.export(placed,str(OUT/f'part_{index:02d}.step'))
(OUT/'parts.json').write_text(json.dumps({'source_url':'https://cad.onshape.com/documents/4c6660b7e2058746f4c9d98e/w/cdc482a4e44c50d42d17047f/e/ae7c642931b4421d81b93862','tab':'REV 2 — PLA Fixture — Assembled','source_units':'mm','simulation_units':'m','parts':parts},indent=2))
