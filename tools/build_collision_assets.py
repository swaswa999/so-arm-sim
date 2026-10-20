"""Generate convex collision pieces and unit-mass inertia from the source CAD.

Only this offline tool needs trimesh/CoACD; the simulator uses checked-in pieces.
"""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
import coacd
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets/physics_collision'
DECOMPOSE = {*(f'tool_mesh_{i}' for i in (3,5,15,18,19,20)), 'tire_tool_part_4', 'tire_tool_part_38', 'docking_guide_mesh', 'adapter_mesh', 'tool_mesh_0', 'tool_mesh_1',
             'tool_mesh_7', 'tool_mesh_14', 'tool_mesh_17',
             *(f'wheel_mesh_{i}' for i in range(7)),
             *(f'tire_tool_part_{i}' for i in (15, 23, 24, 25))}



def axial_pieces(mesh, name):
    # Preserve the measured stepped/tapered bore. Generic hull-count-limited
    # decomposition can bridge this narrow clearance and falsely jam the axle.
    cuts = ([-25.55,-18.,-17.8,-14.8,3.3,4.,9.] if name == 'wheel_mesh_3'
            else [-19.,-18.,-17.,-15.,-7.3,-7.])
    parts = []
    for low, high in zip(cuts,cuts[1:]):
        band = mesh.slice_plane([0,low*.001,0],[0,1,0])
        band = band.slice_plane([0,high*.001,0],[0,-1,0])
        if name == 'wheel_mesh_3':
            hull = band.convex_hull
            parts.append((hull.vertices,hull.faces))
        else:
            for angle in np.arange(32)*2*np.pi/32:
                end = angle+2*np.pi/32
                wedge = band.slice_plane([0,0,.052],[-np.sin(angle),0,np.cos(angle)])
                wedge = wedge.slice_plane([0,0,.052],[np.sin(end),0,-np.cos(end)])
                if len(wedge.vertices) < 4:
                    continue
                hull = wedge.convex_hull
                parts.append((hull.vertices,hull.faces))
    return parts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', nargs='*')
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    manifest_path = OUT / 'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    root = ET.parse(ROOT / 'full_arm.xml').getroot()
    coacd.set_log_level('error')
    for entry in root.findall('./asset/mesh'):
        name = entry.get('name') or Path(entry.get('file')).stem
        if not (name.startswith(('tool_mesh_', 'wheel_mesh_', 'tire_tool_part_'))
                or name in ('adapter_mesh', 'docking_guide_mesh')):
            continue
        if args.only and name not in args.only:
            continue
        source = ROOT / entry.get('file')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        recipe = 'axial-clearance-v1' if name in ('wheel_mesh_3','wheel_mesh_6') else 'capture-clearance-v2' if name in ('tool_mesh_0','tool_mesh_19') else 'coacd-v1'
        if name in manifest and manifest[name]['source_sha256'] == digest and manifest[name].get('recipe','coacd-v1') == recipe and (name not in DECOMPOSE or manifest[name]['collision_method'] != 'convex hull'):
            continue
        mesh = trimesh.load_mesh(source)
        mesh.apply_scale(np.fromstring(entry.get('scale', '1 1 1'), sep=' '))
        solid = mesh if mesh.is_watertight and mesh.volume > 0 else mesh.convex_hull
        props = solid.mass_properties
        mass = float(props['mass'])
        pieces = []
        if recipe == 'axial-clearance-v1':
            parts = axial_pieces(mesh, name)
        elif name in DECOMPOSE:
            settings = dict(threshold=.015,max_convex_hull=40,resolution=1500,
                            mcts_iterations=40,mcts_nodes=12,seed=17,
                            decimate=True,max_ch_vertex=64)
            if recipe == 'capture-clearance-v2':
                settings.update(threshold=.005,max_convex_hull=128,resolution=2500,
                                preprocess_resolution=150)
            parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **settings)
        else:
            hull = mesh.convex_hull
            parts = [(hull.vertices, hull.faces)]
        for i, (vertices, faces) in enumerate(parts):
            filename = f'{name}_{i:02}.stl'
            trimesh.Trimesh(vertices, faces, process=False).export(OUT / filename)
            pieces.append(filename)
        manifest[name] = dict(source=str(source.relative_to(ROOT)), source_sha256=digest, recipe=recipe,
                              pieces=pieces, volume_m3=mass,
                              center_m=np.asarray(props['center_mass']).tolist(),
                              unit_inertia_m2=(np.asarray(props['inertia']) / mass).tolist(),
                              inertia_source='closed CAD volume' if solid is mesh else 'convex hull fallback',
                              collision_method=('axial CAD slicing with 32 bore sectors' if name == 'wheel_mesh_6' else 'axial CAD slicing') if recipe == 'axial-clearance-v1' else 'CoACD approximate convex decomposition' if name in DECOMPOSE else 'convex hull',
                              decomposition_settings=settings if name in DECOMPOSE and recipe != 'axial-clearance-v1' else {})
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
        print(name, len(pieces), 'collision pieces', flush=True)
    used = {filename for item in manifest.values() for filename in item['pieces']}
    for path in OUT.glob('*.stl'):
        if path.name not in used:
            path.unlink()


if __name__ == '__main__':
    main()
