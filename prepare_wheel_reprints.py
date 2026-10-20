from pathlib import Path
import json
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
import trimesh
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'assets/wheel_reprints'
s=cq.importers.importStep(str(OUT/'part_03.step')).val()
for f in s.Faces():
 a=BRepAdaptor_Surface(f.wrapped)
 if a.GetType()==GeomAbs_Cylinder:
  c=a.Cylinder(); print('AXLE CYLINDER diameter',round(2*c.Radius(),4),'axis',c.Axis().Direction().Coord(),'area',round(f.Area(),3))
# Preserve the supplied hollow shape exactly; only recenter and orient head-down.
s=s.translate((0,0,-102)).rotate((0,0,0),(1,0,0),-90)
b=s.BoundingBox();s=s.translate((-(b.xmin+b.xmax)/2,-(b.ymin+b.ymax)/2,-b.zmin))
for ext in ['step','stl']:cq.exporters.export(s,str(OUT/('Hollow_Axle_Rev2_Reprint.'+ext)),tolerance=.015,angularTolerance=.08)
m=trimesh.load(OUT/'Hollow_Axle_Rev2_Reprint.stl');print('AXLE watertight',m.is_watertight,'size',m.extents);assert m.is_watertight and len(m.split())==1
# Inspect fixture planar faces to identify actual ledges/overlap surfaces.
f=cq.importers.importStep(str(OUT/'part_00.step')).val()
for face in f.Faces():
 if face.geomType()=='PLANE' and face.Area()>60:
  print('FIXTURE FACE',round(face.Area(),1),tuple(round(x,2) for x in face.Center().toTuple()),tuple(round(x,2) for x in face.normalAt().toTuple()))
p=json.loads((OUT/'parts.json').read_text());p['tab']='REV 2 — PLA Fixture — Exploded';p['source_url']='https://cad.onshape.com/documents/4c6660b7e2058746f4c9d98e/w/cdc482a4e44c50d42d17047f/e/248544e13a08bb08dc7bb0ff';(OUT/'parts.json').write_text(json.dumps(p,indent=2))
