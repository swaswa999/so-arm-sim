"""Prototype floor-level locator, registered against the wheel base's left edge.
0.35 mm per-side lower clearance; 1 mm upper lead-in. Fit must be tested.
No floor beneath the tool. Source tool CAD placement is preserved.
"""
from pathlib import Path
import json
import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull
import trimesh
from full_arm_preview import DOCKTIP,DOCKROT,TIP
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'assets/docking_guide';OUT.mkdir(exist_ok=True)
# Footprint of the lowest 2.5 mm of the inverted orange housing.
mesh=trimesh.load(ROOT/'assets/socket_tool/part_00.stl')
world=mesh.vertices*.001@DOCKROT.T+(DOCKTIP-DOCKROT@TIP)
points=world[world[:,2]<.0025,:2]*1000
points=points[ConvexHull(points).vertices]
# CadQuery offsets polygon wires to produce a low frame and a tapered cavity.
wire=cq.Workplane('XY').polyline(points.tolist()).close().val()
outer=wire.offset2D(4.0)[0]
inner=wire.offset2D(.35)[0]
upper=wire.offset2D(1.0)[0]
frame=cq.Workplane('XY').add(outer).toPending().extrude(2.5)
# Through opening plus chamfered lead-in in the upper 1 mm.
cut=cq.Workplane('XY').add(inner).toPending().extrude(1.5)
lead=cq.Solid.makeLoft([inner.translate((0,0,1.5)),upper.translate((0,0,2.6))])
frame=frame.cut(cut).cut(lead)
# A square datum edge butts against the wheel fixture's x=-42.5 mm side.
# Connector stays outside that fixture; fasten the guide to the shared board.
base_edge=-42.5
start=points[np.argmax(points[:,0])]+np.array([1.5,0])
end=np.array([base_edge-3,-31.])
direction=end-start;normal=np.array([-direction[1],direction[0]])/np.linalg.norm(direction)*4
corners=[start+normal,start-normal,end-normal,end+normal]
arm=cq.Workplane('XY').polyline([p.tolist() for p in corners]).close().extrude(2.5)
datum=cq.Workplane('XY').box(6,36,2.5,centered=(False,True,False)).translate((base_edge-6,-20,0))
guide=frame.union(arm).union(datum)
# A screw hole in the datum strip holds the locator down after squaring it.
for yy in [-31,-9]:
 hole=cq.Workplane('XY').center(base_edge-3,yy).circle(1.7).extrude(4)
 guide=guide.cut(hole)
assert len(guide.solids().vals())==1, 'Guide pieces must be connected'
cq.exporters.export(guide,str(OUT/'floor_docking_guide.step'))
cq.exporters.export(guide,str(OUT/'floor_docking_guide.stl'),tolerance=.03,angularTolerance=.1)
(OUT/'design.json').write_text(json.dumps({'status':'prototype locator - print fit test required','units':'mm','height':2.5,'lower_clearance_per_side':.35,'upper_clearance_per_side':1.,'datum':'straight right edge against left side of Rev 2 tire base','mounting':'fasten both fixture and guide to one rigid board; two 3.4 mm guide holes provided','tool_support':'tool contacts board through open bottom','limitations':['clearance does not eliminate play; adjust after fit test','no force, friction, tip-over, or release-load validation','CAD housing has approximately 1.49 degree roll relative to socket axis; final seating orientation needs physical confirmation']},indent=2))
print('Exported guide STEP and STL:',len(guide.solids().vals()),'solid(s)')
