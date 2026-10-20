from pathlib import Path
import cadquery as cq,json,math
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'assets/two_tool_guide_rev2'
r=json.loads((OUT/'design.json').read_text());dock=cq.importers.importStep(str(OUT/'Mayter_Socket_and_Tire_Funnel_Dock_Rev2.step')).val()
results=[]
for p in r['placements']:
 s=cq.importers.importStep(str(ROOT/'assets'/p['folder']/'source.step')).val()
 # Rz(theta) Rx(180) Ry(-roll), identical to export placement.
 s=s.rotate((0,0,0),(0,1,0),-r['housing_level_correction_deg']).rotate((0,0,0),(1,0,0),180).rotate((0,0,0),(0,0,1),math.degrees(math.atan2(.21558749,.97648453))).translate(tuple(p['translation_mm']))
 overlap=s.intersect(dock);v=overlap.Volume()
 if v>1e-5:
  cq.exporters.export(overlap,str(OUT/(p['folder']+'_overlap.stl')))
  for solid in overlap.Solids():
   bb=solid.BoundingBox();print('Overlap bounds:',[bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax],flush=True)
 results.append({'assembly':p['folder'],'seated_overlap_mm3':v});print(results[-1],flush=True)
(OUT/'clearance_checks.json').write_text(json.dumps(results,indent=2))
