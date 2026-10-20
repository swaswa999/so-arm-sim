"""Physics-step loop and feedback-gated execution of the old reference path."""
from dataclasses import replace
import math
import mujoco
import numpy as np
from .model import ARM_JOINTS, ROOT, build_model, initialize, parameters
from .servos import PROFILES, ServoBank
from .couplings import ToolLatch, PrintedThread


class Simulation:
    def __init__(self, params=None, scenario='hold', model=None):
        if scenario not in ('hold','cycle','drop'):
            raise ValueError('Unknown scenario')
        self.params = parameters() if params is None else params
        self.model = model if model is not None else build_model(self.params)[0]
        self.data = initialize(self.model)
        self.scenario = scenario
        self.arm_q = np.array([self.model.joint(n).qposadr[0] for n in ARM_JOINTS])
        self.arm_v = np.array([self.model.joint(n).dofadr[0] for n in ARM_JOINTS])
        self.arm_a = np.array([self.model.actuator(n+'_motor').id for n in ARM_JOINTS])
        spec = replace(PROFILES[self.params['servo_profile']],**self.params['servo_controller'])
        self.servos = ServoBank(spec,self.data.qpos[self.arm_q],self.model.opt.timestep)
        self.lock_servo = ServoBank(replace(spec,stall_torque=self.params['locking_torque_limit_nm'],
                                            continuous_torque=min(.15,self.params['locking_torque_limit_nm']),
                                            kp=3.,kd=.06,ki=.2,command_speed=.8),[-.3],self.model.opt.timestep)
        self.latch = ToolLatch(self.model,self.data,self.params)
        self.thread = PrintedThread(self.model,self.data,self.params,enabled=scenario!='drop')
        self.reference_time = 0.
        self.tick = 0
        self.motors_enabled = scenario != 'drop'
        self.status = 'running'
        self.failure = None
        self.failure_stage = None
        self.wait_time = 0.
        self.state = dict(label='Gravity and motor hold',jaw=-.3,tire_open=.006,nut_angle=0.,attached='none')
        self.desired_q = self.data.qpos[self.arm_q].copy()
        self.max_tracking_error = 0.
        self.max_contact_force = 0.
        self.max_penetration = 0.
        self.contact_pairs = {}
        self.peak_torque = np.zeros(5)
        self.saturation_ticks = np.zeros(5,dtype=int)
        self.control_ticks = 0
        self.force_pulse_until = 0.
        if scenario == 'cycle':
            import full_arm_preview as reference
            self.reference = reference
            with np.load(ROOT/'full_cycle_trajectory.npz') as cache:
                self.times,self.positions=cache['t'].copy(),cache['q'].copy()
            self.state = reference.state(0.)
        if scenario == 'drop':
            # Initial-condition change only, to exercise unconstrained free fall.
            for name in ('socket_tool','tire_tool','wheel','nut'):
                q = self.model.joint(name+'_free').qposadr[0]
                self.data.qpos[q+2] += .12
            mujoco.mj_forward(self.model,self.data)
            self.state['label']='Motors off: gravity and contact drop test'

    def fail(self, message):
        if self.failure is None:
            self.failure_stage = self.state['label']
            self.failure = message
            self.status = 'failed'
            self.state['label'] = message

    def reference_update(self):
        if self.scenario != 'cycle' or self.status != 'running':
            return
        dt=self.params['control_period']
        # Allow the free tools to settle before starting a reference trajectory.
        if self.data.time < 1.:
            return
        candidate = min(self.reference_time+dt,self.reference.TOTAL)
        desired = self.reference.state(candidate)
        reasons = []
        error = float(np.max(np.abs(self.desired_q-self.data.qpos[self.arm_q])))
        if error > .16:
            reasons.append('arm cannot track within torque/contact limits')
        if desired['attached'] != 'none' and self.latch.attached != desired['attached']:
            reasons.append('tool capture not physically confirmed')
        if desired['attached']=='none' and self.state['attached']!='none' and self.latch.attached is not None:
            reasons.append('tool did not release')
        if desired['nut_mode']=='socket' and self.thread.engaged:
            reasons.append('nut is still threaded')
        if desired['wheel_mode']=='tire':
            wheel=self.data.xpos[self.model.body('wheel').id]
            tool=self.data.xpos[self.model.body('tire_tool').id]
            rot=self.data.xmat[self.model.body('tire_tool').id].reshape(3,3)
            if np.linalg.norm(wheel-(tool+rot@self.reference.TIP))>.01:
                reasons.append('wheel was not retained by jaw contact')
        if reasons:
            self.wait_time += dt
            if self.wait_time > 3.:
                self.fail('; '.join(reasons))
            return
        self.wait_time=0.
        self.reference_time=candidate
        self.state=desired
        self.desired_q=np.array([np.interp(candidate,self.times,self.positions[:,i]) for i in range(5)])
        if candidate >= self.reference.TOTAL:
            # Reference time alone is not a physical success criterion.
            if all(self.final_state_checks().values()):
                self.status='completed'
            else:
                self.fail('reference ended without a physically installed final state')

    def control(self):
        m,d=self.model,self.data
        d.ctrl[self.arm_a] = self.servos.update(self.desired_q,d.qpos[self.arm_q],d.qvel[self.arm_v],self.motors_enabled)
        lock=m.joint('locking_servo')
        d.ctrl[m.actuator('locking_servo_motor').id] = self.lock_servo.update(
            [self.state['jaw']],d.qpos[lock.qposadr],d.qvel[lock.dofadr],self.motors_enabled)[0]
        for name in ('tire_jaw_slide_15','tire_jaw_slide_25'):
            j=m.joint(name)
            force=1800*(self.state['tire_open']-d.qpos[j.qposadr[0]])-1.2*d.qvel[j.dofadr[0]]
            velocity=d.qvel[j.dofadr[0]]
            available=self.params['jaw_force_limit_n']
            if force*velocity>0:
                available*=max(0.,1-abs(velocity)/self.params['jaw_no_load_speed_m_s'])
            d.ctrl[m.actuator(name+'_motor').id] = np.clip(force,-available,available) if self.motors_enabled else 0
        drive=m.joint('drive_turn')
        # Tool spin direction follows the existing CAD's local drive axis.
        drive_target=-self.state['nut_angle']
        torque=.08*(drive_target-d.qpos[drive.qposadr[0]])-.002*d.qvel[drive.dofadr[0]]
        available=self.params['drive_torque_limit_nm']
        velocity=d.qvel[drive.dofadr[0]]
        if torque*velocity>0:
            available*=max(0.,1-abs(velocity)/self.params['drive_no_load_speed_rad_s'])
        d.ctrl[m.actuator('drive_turn_motor').id]=np.clip(torque,-available,available) if self.motors_enabled else 0
        self.control_ticks+=1
        self.saturation_ticks+=self.servos.saturated
        self.peak_torque=np.maximum(self.peak_torque,np.abs(d.ctrl[self.arm_a]))
        self.max_tracking_error=max(self.max_tracking_error,float(np.max(np.abs(self.desired_q-d.qpos[self.arm_q]))))

    def step(self):
        m,d=self.model,self.data
        if self.tick % round(self.params['control_period']/m.opt.timestep)==0:
            self.reference_update()
        self.control()
        d.qfrc_applied[:]=0
        d.xfrc_applied[:]=0
        self.thread.apply()
        self.latch.update(self.state['jaw']>-.1 and self.motors_enabled)
        if d.time < self.force_pulse_until:
            d.xfrc_applied[m.body('custom_gripper').id,:3]=[0,2,0]
        mujoco.mj_step(m,d)
        self.tick+=1
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('Non-finite physics state')
        if any(w.number for w in d.warning):
            raise RuntimeError('MuJoCo reported a numerical warning')
        if self.tick % 10 == 0:
            force=np.zeros(6)
            for i,c in enumerate(d.contact):
                mujoco.mj_contactForce(m,d,i,force)
                self.max_contact_force=max(self.max_contact_force,float(np.linalg.norm(force[:3])))
                self.max_penetration=max(self.max_penetration,max(0.,-c.dist))
                pair=' / '.join(sorted(m.body(m.geom_bodyid[g]).name for g in (c.geom1,c.geom2)))
                previous=self.contact_pairs.setdefault(pair,dict(peak_force_n=0.,peak_penetration_m=0.))
                previous['peak_force_n']=max(previous['peak_force_n'],float(np.linalg.norm(force[:3])))
                previous['peak_penetration_m']=max(previous['peak_penetration_m'],max(0.,float(-c.dist)))
        return self.data

    def final_state_checks(self):
        m,d=self.model,self.data
        checks=dict(tool_released=self.latch.attached is None,
                    nut_thread_engaged=self.thread.engaged,
                    nut_seated=bool(np.linalg.norm(d.xpos[m.body('nut').id]-[0,-.02165,.054])<.001),
                    wheel_seated=bool(np.linalg.norm(d.xpos[m.body('wheel').id]-[0,-.013,.054])<.003),
                    jaw_open=bool(d.qpos[m.joint('locking_servo').qposadr[0]]<-.12))
        for name in ('socket_tool','tire_tool'):
            bid=m.body(name).id
            angle=np.zeros(3)
            mujoco.mju_subQuat(angle,d.xquat[bid],m.body_quat[bid])
            checks[name+'_parked']=bool(np.linalg.norm(d.xpos[bid]-m.body_pos[bid])<.004 and np.linalg.norm(angle)<.1)
        return checks

    def report(self):
        return dict(scenario=self.scenario,status=self.status,failure=self.failure,failure_stage=self.failure_stage,
                    final_state_checks=self.final_state_checks(),
                    simulation_seconds=float(self.data.time),reference_seconds=self.reference_time,
                    stage=self.state['label'],servo_profile=self.params['servo_profile'],
                    physics_timestep_s=self.model.opt.timestep,
                    peak_joint_torque_nm=dict(zip(ARM_JOINTS,self.peak_torque.tolist())),
                    saturation_fraction=dict(zip(ARM_JOINTS,(self.saturation_ticks/max(1,self.control_ticks)).tolist())),
                    maximum_joint_tracking_error_rad=self.max_tracking_error,
                    sampled_maximum_contact_force_n=self.max_contact_force,
                    sampled_maximum_penetration_m=self.max_penetration,
                    contact_pairs=self.contact_pairs,
                    final_joint_error_rad=dict(zip(ARM_JOINTS,(self.desired_q-self.data.qpos[self.arm_q]).tolist())),
                    peak_latch_force_n=self.latch.peak_force,peak_latch_torque_nm=self.latch.peak_torque,
                    latch_events=self.latch.events,thread_events=self.thread.events,
                    attached_tool=self.latch.attached,nut_thread_engaged=self.thread.engaged,
                    unmeasured_parameters=self.params['provenance'])
