"""Run the SO-101 physics bench, headless checks, or the wheel-service attempt."""
from pathlib import Path
import argparse
import csv
import json
import time
import mujoco
import numpy as np
from physics_sim.model import ARM_JOINTS, parameters
from physics_sim.simulation import Simulation
from physics_sim.servos import PROFILES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=('hold','cycle','drop'),default='hold')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--params',type=Path)
    parser.add_argument('--servo-profile',choices=tuple(PROFILES))
    parser.add_argument('--report',type=Path,default=Path('outputs/physics-report.json'))
    parser.add_argument('--csv',type=Path)
    parser.add_argument('--preview',type=Path,help='Save a rendered frame at the end')
    args=parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds<=0:
        parser.error('--seconds must be finite and positive')
    params=parameters(args.params)
    if args.servo_profile:
        params['servo_profile']=args.servo_profile
    sim=Simulation(params,args.scenario)
    rows=[]
    def step():
        sim.step()
        if args.csv and sim.tick%20==0:
            rows.append([sim.data.time,sim.reference_time,
                         *sim.data.qpos[sim.arm_q],*sim.data.qvel[sim.arm_v],
                         *sim.data.actuator_force[sim.arm_a],sim.data.ncon,
                         sim.latch.attached or '',sim.status])
    started=time.monotonic()
    if args.headless:
        next_message=1.
        while sim.data.time<args.seconds-1e-10:
            step()
            if sim.data.time>=next_message:
                print(f'{sim.data.time:.1f}s | {sim.state["label"]} | contacts {sim.data.ncon}',flush=True)
                next_message+=5.
            if sim.status in ('failed','completed'):
                break
    else:
        from mujoco import viewer
        controls={'paused':True,'reset':False,'pulse':False,'motors':False,'contacts':False,'geometry':False}
        def key(code):
            if code in (ord('P'),32):controls['paused']=not controls['paused']
            elif code==ord('R'):controls['reset']=True
            elif code==ord('F'):controls['pulse']=True
            elif code==ord('M'):controls['motors']=True
            elif code==ord('C'):controls['contacts']=True
            elif code==ord('G'):controls['geometry']=True
        with viewer.launch_passive(sim.model,sim.data,key_callback=key,show_left_ui=False,show_right_ui=False) as window:
            window.cam.lookat[:]=[-.035,-.17,.105]
            window.cam.distance=.85
            window.cam.azimuth=210
            window.cam.elevation=-28
            window.opt.geomgroup[3]=0
            window.opt.geomgroup[4]=0
            previous=time.monotonic()
            accumulator=0.
            while window.is_running():
                now=time.monotonic()
                elapsed=min(now-previous,.05)
                previous=now
                with window.lock():
                    if controls['reset']:
                        fresh=Simulation(params,args.scenario,model=sim.model)
                        mujoco.mj_copyData(sim.data,sim.model,fresh.data)
                        # Keep the viewer's original data object, replacing controllers only.
                        fresh.data=sim.data
                        fresh.latch.data=fresh.data
                        fresh.thread.data=fresh.data
                        sim=fresh
                        controls['reset']=False
                        controls['paused']=True
                        accumulator=0.
                    if controls['pulse']:
                        sim.force_pulse_until=sim.data.time+.2
                        controls['pulse']=False
                    if controls['motors']:
                        sim.motors_enabled=not sim.motors_enabled
                        controls['motors']=False
                    if controls['contacts']:
                        flag=mujoco.mjtVisFlag.mjVIS_CONTACTFORCE
                        window.opt.flags[flag]=not window.opt.flags[flag]
                        controls['contacts']=False
                    if controls['geometry']:
                        window.opt.geomgroup[3]=not window.opt.geomgroup[3]
                        controls['geometry']=False
                    if not controls['paused']:
                        accumulator=min(accumulator+elapsed,.05)
                        frame_steps=0
                        # Keep the integration step fixed even if rendering falls behind.
                        while accumulator>=sim.model.opt.timestep and frame_steps<10:
                            step()
                            frame_steps+=1
                            accumulator-=sim.model.opt.timestep
                            if sim.status=='failed':
                                controls['paused']=True
                                accumulator=0.
                                break
                    window.set_texts((100,0,
                        'MAYTER | PHYSICS\n'+sim.state['label']+'\nP/Space: pause  R: reset  F: push  M: motors  C: forces  G: contacts',
                        f'{sim.data.time:.2f}s | {sim.data.ncon} contacts | {"PAUSED" if controls["paused"] else sim.status}\n'
                        f'Motors {"ON" if sim.motors_enabled else "OFF"} | {params["servo_profile"]} | uncalibrated mass/friction'))
                window.sync()
                time.sleep(.005)
    report=sim.report()
    report['wall_seconds']=time.monotonic()-started
    if sim.status=='running':
        report['status']='horizon_reached'
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    if args.csv:
        args.csv.parent.mkdir(parents=True,exist_ok=True)
        with args.csv.open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time_s','reference_time_s',*[n+'_rad' for n in ARM_JOINTS],
                             *[n+'_rad_s' for n in ARM_JOINTS],*[n+'_Nm' for n in ARM_JOINTS],
                             'contacts','attached_tool','status'])
            writer.writerows(rows)
    if args.preview:
        from PIL import Image
        camera=mujoco.MjvCamera()
        camera.lookat[:]=[-.035,-.17,.105]
        camera.distance=.85
        camera.azimuth=210
        camera.elevation=-28
        option=mujoco.MjvOption()
        option.geomgroup[3:5]=0
        with mujoco.Renderer(sim.model,height=900,width=1200) as renderer:
            renderer.update_scene(sim.data,camera=camera,scene_option=option)
            args.preview.parent.mkdir(parents=True,exist_ok=True)
            Image.fromarray(renderer.render()).save(args.preview)
    print(json.dumps(report,indent=2),flush=True)
    return 2 if sim.status=='failed' else 0


if __name__=='__main__':
    raise SystemExit(main())
