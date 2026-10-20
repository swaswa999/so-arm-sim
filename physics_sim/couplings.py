"""Load-bearing approximations for a mechanical latch and a printed screw.

Neither mechanism teleports objects or prescribes body positions. Their capture
criteria and strength are explicit estimates, not measured hardware properties.
"""
import math
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def descendants(model, name):
    root = model.body(name).id
    result = set()
    for bid in range(1, model.nbody):
        parent = bid
        while parent and parent != root:
            parent = model.body_parentid[parent]
        if parent == root:
            result.add(bid)
    return result


class ToolLatch:
    def __init__(self, model, data, params):
        self.model, self.data, self.params = model, data, params
        self.hand = descendants(model, 'custom_gripper')
        self.tools = {n: descendants(model, n) for n in ('socket_tool', 'tire_tool')}
        self.attached = None
        self.overload_time = 0.
        self.peak_force = self.peak_torque = 0.
        self.events = []
        self.blocked = set()

    def has_contact(self, name):
        for c in self.data.contact:
            a, b = self.model.geom_bodyid[[c.geom1, c.geom2]]
            if c.dist < .0002 and ((a in self.hand and b in self.tools[name]) or
                                 (b in self.hand and a in self.tools[name])):
                return True
        return False

    def alignment(self, name):
        hand, tool = self.model.body('custom_gripper').id, self.model.body(name).id
        distance = np.linalg.norm(self.data.xpos[hand] - self.data.xpos[tool])
        relative = self.data.xmat[hand].reshape(3,3).T @ self.data.xmat[tool].reshape(3,3)
        return float(distance), float(Rotation.from_matrix(relative).magnitude())

    def release(self, reason):
        if self.attached:
            self.data.eq_active[self.model.equality(self.attached+'_latch').id] = False
            self.events.append(dict(time=float(self.data.time), event=reason, tool=self.attached))
            self.attached = None
            self.overload_time = 0.

    def update(self, close_requested):
        m, d, p = self.model, self.data, self.params
        jaw = d.qpos[m.joint('locking_servo').qposadr[0]]
        if not close_requested:
            self.blocked.clear()
            if jaw < -.12:
                self.release('jaw_opened')
        if self.attached:
            eid = m.equality(self.attached+'_latch').id
            rows = (d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (d.efc_id == eid)
            constraint_load=np.zeros(d.nefc)
            constraint_load[rows]=d.efc_force[rows]
            generalized_load=np.zeros(m.nv)
            mujoco.mj_mulJacTVec(m,d,generalized_load,constraint_load)
            bid=m.body(self.attached).id
            dof=m.jnt_dofadr[m.body_jntadr[bid]]
            force=float(np.linalg.norm(generalized_load[dof:dof+3]))
            torque=float(np.linalg.norm(generalized_load[dof+3:dof+6]))
            self.peak_force=max(self.peak_force,force)
            self.peak_torque=max(self.peak_torque,torque)
            overload = force > p['latch_force_limit_n'] or torque > p['latch_torque_limit_nm']
            self.overload_time = self.overload_time + m.opt.timestep if overload else 0.
            if self.overload_time > .02:
                self.blocked.add(self.attached)
                self.release('latch_overload')
            return
        if not close_requested or abs(jaw) > .045:
            return
        for name in self.tools:
            distance, angle = self.alignment(name)
            if (name in self.blocked or distance > p['latch_position_tolerance_m'] or
                    angle > p['latch_angle_tolerance_rad'] or not self.has_contact(name)):
                continue
            hand, tool = m.body('custom_gripper').id, m.body(name).id
            # Spatial velocities are expressed in the same world orientation.
            vhand, vtool = np.zeros(6), np.zeros(6)
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,hand,vhand,0)
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,tool,vtool,0)
            if (np.linalg.norm(vhand[3:]-vtool[3:]) > p['latch_linear_speed_limit_m_s'] or
                    np.linalg.norm(vhand[:3]-vtool[:3]) > p['latch_angular_speed_limit_rad_s']):
                continue
            rot = d.xmat[hand].reshape(3,3)
            pos = rot.T @ (d.xpos[tool]-d.xpos[hand])
            q = Rotation.from_matrix(rot.T @ d.xmat[tool].reshape(3,3)).as_quat()
            eid = m.equality(name+'_latch').id
            m.eq_data[eid, :3] = 0
            m.eq_data[eid, 3:6] = pos
            m.eq_data[eid, 6:10] = np.r_[q[3],q[:3]]
            d.eq_active[eid] = True
            self.attached = name
            self.events.append(dict(time=float(d.time), event='latch_engaged', tool=name,
                                    position_error_m=distance, angle_error_rad=angle))
            break


class PrintedThread:
    """Compliant helical guide, including breakaway friction and finite engagement.

    CAD thread flanks are below the reliable resolution of the convex proxies.
    This reduced model supplies their guide/reaction forces. It releases after
    the modeled engagement length and only recaptures an aligned turning nut.
    """
    def __init__(self, model, data, params, enabled=True):
        self.model, self.data, self.params = model, data, params
        self.bid = model.body('nut').id
        self.origin = np.array([0., -.02165, .054])
        self.axis = np.array([0., -1., 0.])
        self.angle = self.last_angle = 0.
        self.last_time = float(data.time)
        self.spin_dof = model.joint('nut_free').dofadr[0] + 4
        self.geoms = np.flatnonzero((model.geom_bodyid == self.bid) & (model.geom_contype == 2))
        model.dof_frictionloss[self.spin_dof] = 0.
        # A soft default friction constraint leaks enough torque to spin this
        # tiny inertia below breakaway. Use near-rigid, implicit stiction.
        model.dof_solimp[self.spin_dof] = [.9999,.9999,.001,.5,2.]
        model.dof_solref[self.spin_dof] = [.002,1.]
        model.geom_conaffinity[self.geoms] = 1 if enabled else 5
        self.engaged = enabled
        self.enabled = enabled
        self.events = []

    def apply(self):
        if not self.enabled:
            return
        m, d, p = self.model, self.data, self.params
        # mj_step leaves Cartesian fields at the pre-integration configuration.
        # Refresh them before evaluating a stiff force law from the new q state.
        mujoco.mj_kinematics(m,d)
        mujoco.mj_comPos(m,d)
        mujoco.mj_comVel(m,d)
        rot = d.xmat[self.bid].reshape(3,3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,self.bid,velocity,0)
        spin = np.dot(velocity[:3],self.axis)
        x = rot[:,0]
        wrapped = math.atan2(np.dot(self.axis, np.cross([1,0,0],x)), x[0])
        increment = math.atan2(math.sin(wrapped-self.last_angle), math.cos(wrapped-self.last_angle))
        # Recover complete turns if a very light nut rotates more than pi in a
        # step. MuJoCo integrates orientation with the current post-step speed.
        predicted = spin * (d.time-self.last_time)
        increment += 2*math.pi*round((predicted-increment)/(2*math.pi))
        self.angle += increment
        self.last_angle = wrapped
        self.last_time = float(d.time)
        displacement = d.xpos[self.bid] - self.origin
        axial = np.dot(displacement, self.axis)
        lateral = displacement - axial*self.axis
        tilt = np.cross(rot @ self.axis, self.axis)
        speed = np.dot(velocity[3:],self.axis)
        lead = p['thread_pitch_m'] / (2*math.pi)
        if self.engaged and axial > p['thread_engagement_m']-.0002 and self.angle*lead >= p['thread_engagement_m']:
            self.engaged = False
            self.events.append(dict(time=float(d.time),event='nut_unthreaded'))
        elif (not self.engaged and p['thread_engagement_m']-.0005 < axial < p['thread_engagement_m']+.0005
              and np.linalg.norm(lateral)<.0003 and np.linalg.norm(tilt)<.05 and spin<-.2):
            self.engaged = True
            self.angle = axial/lead
            self.events.append(dict(time=float(d.time),event='thread_recaptured'))
        m.geom_conaffinity[self.geoms] = 1 if self.engaged else 5
        if not self.engaged:
            m.dof_frictionloss[self.spin_dof] = 0.
            return
        # Semi-implicit penalty force stabilisation for this small 2 g nut.
        mass, dt = m.body_mass[self.bid], m.opt.timestep
        stiffness, damping = 1500., .7
        scale = 1 + dt*damping/mass + dt*dt*stiffness/mass
        force = (-stiffness*lateral-(damping+dt*stiffness)*(velocity[3:]-speed*self.axis))/scale
        axial_force = (-stiffness*(axial-lead*self.angle)-(damping+dt*stiffness)*(speed-lead*spin))/scale
        friction = (p['thread_running_nm'] + (p['thread_breakaway_nm']-p['thread_running_nm'])*math.exp(-(spin/.3)**2))
        # Use MuJoCo's implicit friction constraint rather than an explicit
        # sign torque, which would reverse this tiny nut in one integration step.
        # The aligned thread's axis is the nut's local Y rotational degree.
        m.dof_frictionloss[self.spin_dof] = friction
        spin_torque = -lead*axial_force
        if axial < 0:
            axial_force += max(0.,(-stiffness*axial-(damping+dt*stiffness)*speed)/scale)
        transverse_w = velocity[:3]-spin*self.axis
        moment = max(float(m.body_inertia[self.bid].min()),1e-9)
        angular_scale = 1+dt*.0002/moment+dt*dt*.02/moment
        torque = (.02*tilt-.0002*transverse_w)/angular_scale+spin_torque*self.axis
        # Wrench acts at the body's origin. Apply through mj_applyFT so eccentric
        # CAD center of mass receives the physically correct moment.
        mujoco.mj_applyFT(m,d,force+axial_force*self.axis,torque,d.xpos[self.bid],
                         self.bid,d.qfrc_applied)
