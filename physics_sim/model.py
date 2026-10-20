"""Build a dynamic scene without changing the legacy kinematic model."""
from pathlib import Path
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def numbers(values):
    return ' '.join(f'{v:.12g}' for v in values)


def rotation(quat):
    q = np.fromstring(quat or '1 0 0 0', sep=' ')
    return Rotation.from_quat(np.r_[q[1:], q[0]]).as_matrix()


def parameters(path=None):
    params = json.loads(Path(path or Path(__file__).with_name('parameters.json')).read_text())
    dt, period = params['timestep'], params['control_period']
    if not (0 < dt <= .002 and period >= dt and math.isclose(period / dt, round(period / dt))):
        raise ValueError('Use timestep <= 0.002 s and an integer number of physics steps per control period')
    if any(not math.isfinite(v) or v <= 0 for v in params['body_mass_kg'].values()):
        raise ValueError('Body masses must be finite and positive')
    positive = ('contact_time_constant','jaw_force_limit_n','locking_torque_limit_nm',
                'drive_torque_limit_nm','latch_position_tolerance_m','latch_angle_tolerance_rad',
                'latch_force_limit_n','latch_torque_limit_nm','thread_pitch_m','thread_engagement_m',
                'thread_breakaway_nm','thread_running_nm')
    if any(not math.isfinite(params[k]) or params[k] <= 0 for k in positive):
        raise ValueError('Contact and mechanism parameters must be finite and positive')
    if params['servo_profile'] not in ('7.4v','12v'):
        raise ValueError('Unknown servo profile')
    if len(params['friction']) != 3 or any(not math.isfinite(v) or v < 0 for v in params['friction']):
        raise ValueError('Friction must contain three finite nonnegative values')
    if len(params['gravity']) != 3 or not np.isfinite(params['gravity']).all():
        raise ValueError('Invalid gravity vector')
    return params


def cad_inertia(body, mass, manifest):
    """Aggregate the CAD parts with uniform density, then scale to an explicit mass."""
    parts = []
    for geom in body.findall('geom'):
        source = manifest.get(geom.get('mesh'))
        if source is None:
            continue
        rot = rotation(geom.get('quat'))
        pos = np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
        center = rot @ np.array(source['center_m']) + pos
        parts.append((source['volume_m3'], center, rot @ np.array(source['unit_inertia_m2']) @ rot.T))
    if not parts:
        raise ValueError(f'No mass geometry for {body.get("name")}')
    volume = sum(p[0] for p in parts)
    center = sum(v*c for v,c,_ in parts) / volume
    inertia = np.zeros((3, 3))
    for v, c, tensor in parts:
        d = c - center
        inertia += mass * v / volume * (tensor + np.eye(3) * (d @ d) - np.outer(d, d))
    if np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('CAD inertia is not positive definite')
    for old in body.findall('inertial'):
        body.remove(old)
    ET.SubElement(body, 'inertial', pos=numbers(center), mass=str(mass),
                  fullinertia=numbers([inertia[0,0], inertia[1,1], inertia[2,2],
                                       inertia[0,1], inertia[0,2], inertia[1,2]]))


def build_model(params=None):
    params = parameters() if params is None else copy.deepcopy(params)
    root = ET.parse(ROOT / 'full_arm.xml').getroot()
    manifest_path = ROOT / 'assets/physics_collision/manifest.json'
    if not manifest_path.exists():
        raise FileNotFoundError('Missing collision assets; run tools/build_collision_assets.py')
    manifest = json.loads(manifest_path.read_text())
    for name, item in manifest.items():
        source = ROOT / item['source']
        if hashlib.sha256(source.read_bytes()).hexdigest() != item['source_sha256']:
            raise ValueError(f'Stale collision geometry for {name}; rebuild collision assets')
    world = root.find('worldbody')
    compiler = root.find('compiler')
    compiler.set('inertiafromgeom', 'false')
    opt = root.find('option')
    opt.attrib.update(gravity=numbers(params['gravity']), timestep=str(params['timestep']),
                      integrator='implicitfast', solver='Newton', iterations='80',
                      tolerance='1e-9', cone='elliptic')
    ET.SubElement(opt, 'flag', energy='enable')
    for tag in ('actuator', 'equality', 'keyframe', 'contact', 'sensor'):
        for el in root.findall(tag):
            root.remove(el)
    asset = root.find('asset')
    for mesh in asset.findall('mesh'):
        mesh.set('file', str(ROOT / mesh.get('file')))
    bodies = {b.get('name'): b for b in world.iter('body')}
    # The old stock gripper was replaced by custom CAD: avoid counting it twice.
    for inertia in bodies['gripper'].findall('inertial'):
        bodies['gripper'].remove(inertia)
    for name, mass in params['body_mass_kg'].items():
        cad_inertia(bodies[name], mass, manifest)
    layout = json.loads((ROOT / 'assets/sim_two_tool_dock/design.json').read_text())
    for placement in layout['placements']:
        body = bodies[placement['folder']]
        body.attrib.pop('mocap', None)
        body.set('pos', numbers(np.array(placement['translation_mm']) * .001))
        q = Rotation.from_matrix(placement['rotation']).as_quat()
        body.set('quat', numbers(np.r_[q[3], q[:3]]))
        ET.SubElement(body, 'freejoint', name=placement['folder']+'_free')
    fixture = bodies['wheel_fixture']
    for name in ('wheel', 'nut'):
        body = bodies[name]
        fixture.remove(body)
        world.append(body)
        body.set('pos', numbers(np.fromstring(body.get('pos'), sep=' ')+[0,0,.002]))
        for joint in body.findall('joint'):
            body.remove(joint)
        ET.SubElement(body, 'freejoint', name=name+'_free')
    for joint in world.iter('joint'):
        joint.set('damping', str(params['joint_damping']))
        joint.set('frictionloss', str(params['joint_friction']))
        joint.set('armature', str(params['reflected_motor_inertia']))
        if joint.get('type') == 'slide':
            joint.attrib.update(damping='.2', frictionloss='.05', armature='.001')
        if joint.get('name') == 'drive_turn':
            joint.attrib.update(damping='.0002', frictionloss='.0001', armature='.00001')
    # Separate visual meshes from collision geometry; CAD cavities use decomposition.
    for index, (parent, geom) in enumerate([(p, g) for p in world.iter() for g in p.findall('geom')]):
        geom.set('mass', '0')
        if geom.get('name') == 'table' or geom.get('class') == 'collision':
            geom.attrib.update(contype='1', conaffinity='1', condim='4',
                               friction=numbers(params['friction']),
                               solref=f'{params["contact_time_constant"]} 1')
            if geom.get('name') != 'table':
                geom.set('rgba', '.2 .7 .9 .25')
                geom.set('name', f'arm_collision_{index}')
            continue
        geom.attrib.update(contype='0', conaffinity='0', group='2')
        source = manifest.get(geom.get('mesh'))
        if source is None:
            continue
        for i, filename in enumerate(source['pieces']):
            mesh_name = f'physics_{geom.get("mesh")}_{i}'
            if asset.find(f'mesh[@name="{mesh_name}"]') is None:
                ET.SubElement(asset, 'mesh', name=mesh_name,
                              file=str(ROOT / 'assets/physics_collision' / filename))
            collision = copy.deepcopy(geom)
            collision.attrib.pop('material', None)
            collision.attrib.pop('class', None)
            collision.attrib.update(name=f'contact_{index}_{i}', mesh=mesh_name,
                                    group='3', rgba='.2 .7 .9 .25', contype='1', conaffinity='1',
                                    condim='4', friction=numbers(params['friction']),
                                    solref=f'{params["contact_time_constant"]} 1',
                                    solimp='.9 .95 .001', margin='.0001', gap='0')
            parent.append(collision)
    actuator = ET.SubElement(root, 'actuator')
    for name in (*ARM_JOINTS, 'locking_servo', 'drive_turn', 'tire_jaw_slide_15', 'tire_jaw_slide_25'):
        limit = (3. if name in ARM_JOINTS else params['locking_torque_limit_nm']
                 if name == 'locking_servo' else params['drive_torque_limit_nm']
                 if name == 'drive_turn' else params['jaw_force_limit_n'])
        ET.SubElement(actuator, 'motor', name=name+'_motor', joint=name,
                      ctrlrange=f'{-limit} {limit}', forcerange=f'{-limit} {limit}')
    equality = ET.SubElement(root, 'equality')
    for name in ('socket_tool', 'tire_tool'):
        ET.SubElement(equality, 'weld', name=name+'_latch', body1='custom_gripper',
                      body2=name, active='false', solref='.008 1', torquescale='.03')
    xml = ET.tostring(root, encoding='unicode')
    model = mujoco.MjModel.from_xml_string(xml)
    return model, xml


def initialize(model):
    data = mujoco.MjData(model)
    # Initial state only; subsequent movement must come from mj_step and forces.
    with np.load(ROOT / 'full_cycle_trajectory.npz') as cache:
        q = cache['q'][0].copy()
    for name, value in zip(ARM_JOINTS, q):
        data.qpos[model.joint(name).qposadr[0]] = value
    data.qpos[model.joint('locking_servo').qposadr[0]] = -.3
    for name in ('tire_jaw_slide_15', 'tire_jaw_slide_25'):
        data.qpos[model.joint(name).qposadr[0]] = .006
    mujoco.mj_forward(model, data)
    return data
