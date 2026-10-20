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


def test_thread_friction_dissipates_spin_instead_of_reversing_it():
    xml='''<mujoco><option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
    <worldbody><body name="nut" pos="0 -.02165 .054"><freejoint name="nut_free"/>
    <inertial pos="0 0 0" mass=".002" diaginertia="4e-8 4e-8 4e-8"/>
    <geom type="sphere" size=".005" contype="0" conaffinity="0"/>
    </body></worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
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
