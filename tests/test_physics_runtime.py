import copy
import mujoco
import numpy as np
import pytest
from physics_sim.model import build_model, parameters
from physics_sim.simulation import Simulation
from physics_sim.couplings import PrintedThread


@pytest.fixture(scope='module')
def model():
    return build_model()[0]


def test_runtime_cannot_move_objects_without_the_integrator(model,monkeypatch):
    sim=Simulation(model=model)
    before_q, before_v=sim.data.qpos.copy(),sim.data.qvel.copy()
    before_pos,before_quat=model.body_pos.copy(),model.body_quat.copy()
    monkeypatch.setattr(mujoco,'mj_step',lambda *args:None)
    for _ in range(50):sim.step()
    np.testing.assert_array_equal(before_q,sim.data.qpos)
    np.testing.assert_array_equal(before_v,sim.data.qvel)
    np.testing.assert_array_equal(before_pos,model.body_pos)
    np.testing.assert_array_equal(before_quat,model.body_quat)
    assert sim.data.time==0


def test_hold_tracks_with_real_motor_torques(model):
    sim=Simulation(model=model)
    for _ in range(1200):sim.step()
    assert np.max(np.abs(sim.desired_q-sim.data.qpos[sim.arm_q]))<.03
    assert np.any(np.abs(sim.data.actuator_force[sim.arm_a])>.01)
    assert np.all(sim.peak_torque<=sim.servos.spec.stall_torque+1e-8)
    assert not any(w.number for w in sim.data.warning)


def test_open_or_distant_gripper_does_not_capture_tool(model):
    sim=Simulation(model=model)
    sim.latch.update(True)
    assert sim.latch.attached is None
    sim.data.qpos[model.joint('locking_servo').qposadr[0]]=0
    mujoco.mj_forward(model,sim.data)
    sim.latch.update(True)
    assert sim.latch.attached is None
    assert not sim.data.eq_active.any()


def test_motor_disable_removes_all_actuator_effort(model):
    sim=Simulation(model=model)
    for _ in range(20):sim.step()
    sim.motors_enabled=False
    sim.step()
    np.testing.assert_allclose(sim.data.actuator_force,0,atol=1e-12)


def isolated_nut():
    xml='''<mujoco><option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
    <worldbody><body name="nut" pos="0 -.02165 .054"><freejoint name="nut_free"/>
    <inertial pos="0 0 0" mass=".002" diaginertia="4e-8 4e-8 4e-8"/>
    <geom type="sphere" size=".005" contype="0" conaffinity="0"/>
    </body></worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    return m,d


def test_thread_friction_dissipates_spin_instead_of_reversing_it():
    m,d=isolated_nut()
    d.qvel[4]=-1.
    mujoco.mj_forward(m,d)
    thread=PrintedThread(m,d,parameters())
    speeds=[]
    for _ in range(100):
        d.qfrc_applied[:]=0
        thread.apply()
        mujoco.mj_step(m,d)
        speeds.append(d.qvel[4])
    assert max(abs(v) for v in speeds)<=1.01
    assert abs(speeds[-1])<.05
    assert np.isfinite(d.qpos).all()


@pytest.mark.parametrize('torque', [.005, .01])
def test_thread_resists_load_below_breakaway(torque):
    m,d=isolated_nut()
    thread=PrintedThread(m,d,parameters())
    for _ in range(500):
        d.qfrc_applied[:]=0
        thread.apply()
        d.qfrc_applied[4]-=torque
        mujoco.mj_step(m,d)
    assert thread.engaged
    assert abs(thread.angle)<.02
    assert abs(thread.origin[1]-d.qpos[1])<3e-6


def test_thread_releases_under_torque_after_finite_engagement():
    m,d=isolated_nut()
    thread=PrintedThread(m,d,parameters())
    for _ in range(500):
        d.qfrc_applied[:]=0
        thread.apply()
        if not thread.engaged:
            break
        # A bare, low-inertia nut accelerates rapidly without an attached drive.
        d.qfrc_applied[4]-=.03
        mujoco.mj_step(m,d)
    assert not thread.engaged
    assert thread.angle>=8*2*np.pi
    assert .0078 < thread.origin[1]-d.qpos[1] < .01
    assert any(event['event']=='nut_unthreaded' for event in thread.events)
    assert not any(w.number for w in d.warning)


@pytest.mark.parametrize('close_requested', [True, False])
def test_contact_capture_preserves_pose_and_overload_breaks_the_latch(close_requested):
    from physics_sim.couplings import ToolLatch
    xml='''<mujoco><option timestep=".001" gravity="0 0 0"/>
    <default><geom margin=".0001"/></default><worldbody>
      <body name="custom_gripper" pos="0 0 .2">
        <geom type="box" pos="-.01 0 0" size=".005 .005 .005"/>
        <body name="locking_jaw" pos="0 .03 0">
          <joint name="locking_servo" range="-.35 .1"/>
          <inertial pos="0 0 0" mass=".01" diaginertia="1e-5 1e-5 1e-5"/>
        </body>
      </body>
      <body name="socket_tool" pos="0 0 .2"><freejoint/>
        <geom type="box" size=".005 .005 .005" mass=".02"/>
      </body>
      <body name="tire_tool" pos="1 0 .2"><freejoint/>
        <geom type="box" size=".005 .005 .005" mass=".02"/>
      </body>
    </worldbody><equality>
      <weld name="socket_tool_latch" body1="custom_gripper" body2="socket_tool" active="false" solref=".008 1" torquescale=".03"/>
      <weld name="tire_tool_latch" body1="custom_gripper" body2="tire_tool" active="false" solref=".008 1" torquescale=".03"/>
    </equality></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    latch=ToolLatch(m,d,parameters())
    before=d.qpos.copy()
    latch.update(True)
    assert latch.attached=='socket_tool'
    np.testing.assert_array_equal(d.qpos,before)
    # A continuous pull above the configured latch rating must break it.
    for _ in range(150):
        d.xfrc_applied[m.body('socket_tool').id,0]=100.
        mujoco.mj_step(m,d)
        latch.update(close_requested)
    assert latch.attached is None
    assert any(event['event']=='latch_overload' for event in latch.events)
    assert not d.eq_active.any()
