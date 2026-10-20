MAYTER SOCKET + TIRE TOOL DOCK — REV 2

Import Mayter_Socket_and_Tire_Funnel_Dock_Rev2.step into Onshape.
STL is supplied for a prototype print. Units: millimetres.

RIGHT BAY: socket tool. Lower opening approximately 45.1 x 48.8 mm.
LEFT BAY: tire tool. Lower opening approximately 53.7 x 53.8 mm.
Both pockets: 18 mm tall; 3 mm straight locating section; open gripper approach side.
Sloping lead-in: 10 mm per side for socket, 15 mm per side for tire.
Nominal lower-pocket clearance: 0.6 mm per side.
Bay centres: 95 mm apart. Overall size approximately 199 x 103 x 18 mm.
The pocket dimensions come from each housing's lower 3 mm in its parked orientation,
not the overall tool bounding box.

Orange housing faces are levelled by the measured 1.493 degree CAD tilt before
placing them on the board. Tools rest on the board through the open bottoms.
Mount the straight datum tab against the left edge of the wheel fixture and
fasten both the fixture and dock to the same rigid board.

The tire tool's wider sloping side plates required a broader funnel than the
socket tool. The final fit check uses complete imported assemblies in their
supplied poses, including the attached gripper. See clearance_checks.json.

Not yet validated: physical friction, print tolerances, automatic seating from
misaligned drops, jaw travel beyond the supplied pose, or the full robot approach
and release path. An open-front dock cannot constrain sliding toward that opening:
the arm must seat the tool toward the closed stop before release. Test by hand first.
The existing animated robot trajectory has not been changed to these new dock poses.

SOURCE: Onshape document 2ac14cfd423a4120f8cad5e6, workspace acc8ed4486356bf30f43548c.
Tire tool assembly: element 912d94122f10ecaac60cf653, exported October 15, 2026.
Socket tool: prior supplied export, element f39f09d551100f323b4cfe55.
