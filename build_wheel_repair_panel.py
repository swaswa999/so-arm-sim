from pathlib import Path
import cadquery as cq
import trimesh,json
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'assets/wheel_reprints'
# Rear face of fixture: lower bridge y=13.8, upper block y=12.
# Leave 0.2 mm for adhesive; foot rests above the base at z=6.
web=cq.Workplane('XY').box(30,3,50.8,centered=(True,False,False)).translate((0,14,6.2))
foot=cq.Workplane('XY').box(30,10,3,centered=(True,False,False)).translate((0,14,6.2))
pad=cq.Workplane('XY').box(28,1.8,10,centered=(True,False,False)).translate((0,12.2,47))
repair=web.union(foot).union(pad)
assert repair.val().isValid() and len(repair.solids().vals())==1
# Save fitted placement for inspection; printable copy lies on the flat rear web.
cq.exporters.export(repair,str(OUT/'Glue_Panel_Fitted_Position.step'))
cq.exporters.export(repair,str(OUT/'Glue_Panel_Fitted_Position.stl'),tolerance=.02)
# Print on a side edge so the full L-shaped web and foot touch the bed.
flat=repair.rotate((0,0,0),(0,1,0),90)
b=flat.val().BoundingBox();flat=flat.translate((-(b.xmin+b.xmax)/2,-(b.ymin+b.ymax)/2,-b.zmin))
for ext in ['step','stl']:cq.exporters.export(flat,str(OUT/('Wheel_Fixture_Glue_Repair_Panel.'+ext)),tolerance=.02)
m=trimesh.load(OUT/'Wheel_Fixture_Glue_Repair_Panel.stl');assert m.is_watertight and len(m.split())==1
fixture=cq.importers.importStep(str(OUT/'part_00.step')).val()
overlap=fixture.intersect(repair.val()).Volume()
report={'units':'mm','panel_width':30,'panel_height':50.8,'web_thickness':3,'foot_depth':10,'foot_thickness':3,'upper_pad_width':28,'upper_pad_height':10,'nominal_glue_gap':.2,'intact_fixture_overlap_mm3':overlap,'watertight':bool(m.is_watertight),'limitations':['Rigid repair removes the original support flexure behavior.','Broken-stub clearance and actual print fit require dry fitting.','Adhesive strength has not been load-tested.']}
(OUT/'repair_panel_dimensions.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
assert overlap<.001
