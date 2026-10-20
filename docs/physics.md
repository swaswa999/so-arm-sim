# Physics model and calibration

`run_physics.py` runs an independent MuJoCo dynamics model. The original
`full_arm_preview.py` remains the kinematic reference and is not evidence of
physical success.

## What changes

- Gravity acts on the arm, tools, wheel and nut. Tools and workpieces have free
  joints; their positions and velocities are initialized once, then integrated
  by `mj_step`.
- Arm motors apply bounded torques. References arrive at 100 Hz with modeled
  communication delay; the internal servo feedback loop runs every 1 ms physics
  step. It models encoder resolution, command slew, approximate servo PID,
  motor torque/speed falloff, and estimated sustained-load derating.
- Visual meshes do not contribute extra mass or collision forces. Upstream arm
  inertias are retained through the wrist; custom assemblies have explicit
  masses and CAD-derived centers/tensors. The obsolete stock gripper inertia is
  removed to avoid counting both grippers.
- Contact uses friction and compliant constraints. Concave CAD geometry is
  approximated by convex pieces, preserving openings more accurately than a
  single hull. The original CAD remains the visible geometry.
- Mechanical tool capture uses an initially disabled soft weld. It can engage
  only with an actually closed jaw, nearby aligned frames, low relative motion,
  and contact between the hand and tool. Capture preserves the measured relative
  pose; it does not move the tool. Excess constraint load breaks the latch.
- Tire jaws and the socket drive have force/torque and speed limits. Wheel pickup relies on collision and friction;
  there is no wheel-to-tool weld or timed wheel position override.
- The printed nut uses a compliant helical guide with pitch, finite engagement,
  and estimated breakaway/running resistance. It disengages after the thread
  travel and may recapture during an aligned reverse turn.

## Run

```sh
python -m pip install -r requirements.txt
# macOS interactive viewer
.venv/bin/mjpython run_physics.py
# Other desktop platforms
python run_physics.py
# Diagnostics without a viewer
python run_physics.py --headless --scenario hold --seconds 5
python run_physics.py --headless --scenario drop --seconds 2
python run_physics.py --headless --scenario cycle --seconds 180 \
  --csv outputs/cycle.csv --report outputs/cycle.json
```

The viewer starts paused. **P** or **Space** runs/pauses, **R** resets, **M**
toggles motor power, and **F** applies a 2 N disturbance for 0.2 seconds. **C** toggles contact-force
arrows and **G** toggles the collision pieces. All
controls affect only the simulation. Rendering does not change the physics
step size. If the simulation is slower than real time, the displayed simulation
clock is the authoritative clock.

`--preview outputs/physics.png` saves the final frame. The default profile is
`hiwonder-hx30hm`; `--servo-profile 7.4v` and `--servo-profile 12v` select the
earlier Feetech ratings. `--params path/to/parameters.json` loads
a complete calibration file copied from `physics_sim/parameters.json`.

The cycle runner uses the old trajectory as a *request* to the motors. It waits
when tracking, tool capture, nut release or wheel retention are not confirmed,
and reports a failure if those conditions do not recover. Reaching the end of
a requested time horizon is not reported as a completed service cycle. Exit
code 2 means a physical execution gate failed; numerical failures raise an
error separately.

JSON reports include the parameter values and MuJoCo version used, joint
tracking errors, torque peaks, saturation fractions, and latch/thread events.
Contact peaks are sampled every ten physics steps; they can miss shorter impact
spikes. CSV output records joint positions, velocities and actuator torques.

## What is known and what must be measured

| Quantity | Basis |
| --- | --- |
| Arm geometry and link inertia | Included SO-101 model and its upstream documentation |
| Custom geometry | Included CAD, converted from millimetres to metres |
| Default servo stall torque and speed | HiWonder HX-30HM: 30 kgf cm and 0.19 s/60 degrees, specified at 11.1 V |
| Sustained torque allowance | HX-30HM: unmeasured one-third-stall estimate, not a manufacturer continuous rating; Feetech alternatives use published rated-load values as an assumption |
| Custom mass, friction, contact compliance | Explicit estimates in the parameter file |
| Servo gains, latency, damping, rotor inertia, derating | Identification estimates |
| Tool latch strength and capture tolerance | Estimates, not a validated mechanical latch design |
| Nut resistance, pitch engagement and thread recapture | Reduced model of the small printed demonstration fastener |

The default assumes HiWonder's HX-30HM follower with 1/345 gearing, inferred
from the user's kit brand. Its exact installed motor variant remains unconfirmed.
HiWonder lists a 12 V supply for the kit, but specifies the motor endpoints at
11.1 V; the simulation uses those published endpoints without voltage scaling.
Encoder command resolution is 4,096 counts; this does not imply equivalent
mechanical accuracy. Leader-arm motors differ. Custom locking, socket-drive,
and tire-jaw mechanisms retain separate estimated limits.

Measure the complete socket/tire tools and custom gripper on a scale first.
Record servo model and supply voltage, then compare loaded static positions,
step responses, and lift limits. Measure pull-out force for the tool latch and
breakaway torque for the printed nut. Use those results to replace the explicit
estimates; changing gains merely to force a successful cycle is not calibration.

The contact decomposition has finite geometric resolution and a hull-count cap.
It does not reproduce exact threads, elastic jaws, gear backlash, plastic
flexure, cable forces, motor heating, thread stripping, or manufacturing
tolerances. While threaded, the reduced guide replaces nut/axle mesh contact to avoid
double-counting resistance. Other fixture contacts stay active. Fastener behavior
still requires dedicated validation. The model does not establish that the physical
robot can complete this service task, and it models a small printed wheel,
not a full-size vehicle wheel.

Physics mode moves the separate release/strip station 15 cm to the right using
`release_station_offset_m`, because its original position intersects the parked
tire tool. This proposed bench layout must be checked against the real hardware;
the original CAD and kinematic scene are unchanged.

## Rebuild collision geometry and test

```sh
python -m pip install -r requirements-dev.txt
python tools/build_collision_assets.py
python -m pytest -q
```

Collision pieces and their source hashes, mass geometry and method are checked
in under `assets/physics_collision/`. They are generated offline, so running the
simulator does not need a CAD kernel or decomposition library.

## References

- [HiWonder SO-ARM101 manual and motor ratings](https://docs.hiwonder.com/projects/LeRobot/en/latest/docs/SO_ARM101_Open_Source_6_Axis_Robotic_Arm_User_Manual.html)
- [HiWonder HX-30HM specifications](https://www.hiwonder.com/products/hx-30hm)
- [Waveshare ST3215 specifications](https://docs.waveshare.com/ST3215_Servo)
- [LeRobot SO-101 assembly and follower gearing](https://huggingface.co/docs/lerobot/main/assemble_so101)
- [MuJoCo contact, actuator, and equality-constraint reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
- [MuJoCo dynamics and constraint solver](https://mujoco.readthedocs.io/en/latest/computation/index.html)
- [CoACD collision decomposition](https://github.com/SarahWeiii/CoACD)
- Included `assets/robotstudio_so101/README.md` and `LICENSE`
