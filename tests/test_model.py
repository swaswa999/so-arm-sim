import mujoco
import numpy as np
import pytest
from physics_sim.model import build_model, initialize, parameters


@pytest.fixture(scope='module')
def model():
    return build_model()[0]


def test_model_has_gravity_free_objects_and_positive_inertias(model):
    assert np.allclose(model.opt.gravity,[0,0,-9.81])
    assert model.nmocap == 0
    assert model.nu == 9
    for name in ('socket_tool','tire_tool','wheel','nut'):
        joint=model.joint(name+'_free')
        assert joint.type[0] == mujoco.mjtJoint.mjJNT_FREE
        body=model.body(name)
        assert body.mass[0]>0 and np.all(body.inertia>0)
    assert np.count_nonzero(model.geom_contype)>20


def test_free_tool_accelerates_under_gravity_without_a_pose_driver(model):
    data=initialize(model)
    joint=model.joint('socket_tool_free')
    q,v=joint.qposadr[0],joint.dofadr[0]
    data.qpos[q:q+3]=[.8,.5,.7]
    mujoco.mj_forward(model,data)
    for _ in range(40):
        mujoco.mj_step(model,data)
    assert data.qvel[v+2] == pytest.approx(-9.81*.04,abs=.002)
    assert .690 < data.qpos[q+2] < .695
    assert np.isfinite(data.qpos).all()


def test_ground_supports_a_free_wheel_with_contact(model):
    data=initialize(model)
    q=model.joint('wheel_free').qposadr[0]
    data.qpos[q:q+3]=[.6,.5,.08]
    mujoco.mj_forward(model,data)
    wheel=model.body('wheel').id
    contact_seen=False
    for _ in range(600):
        mujoco.mj_step(model,data)
        contact_seen |= any(wheel in model.geom_bodyid[[c.geom1,c.geom2]] for c in data.contact)
    assert contact_seen
    assert .010 < data.xpos[wheel,2] < .025
    assert np.isfinite(data.qvel).all()
    assert not any(w.number for w in data.warning)
