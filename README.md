# Franka experiments

`franka_state` reads the arm through the Franka Control Interface (FCI) using
libfranka. The program opens a read-only session to read robot state and query the model.

## Build and run

Use the libfranka release compatible with your robot's system software. The
program was built with libfranka 0.18.2. Enable FCI and use the robot's FCI IP
address or hostname.

If the workstation needs an additional IPv4 address for the FCI connection,
run `./scripts/setup_franka_network.sh`.

On this workstation, add the Pinocchio library directory before building and running.

```bash
source scripts/franka_env.sh
cmake -S . -B build
cmake --build build

# Read one snapshot.
./build/franka_state 172.16.0.2

# Capture 100 snapshots with a 100 ms target interval.
./build/franka_state --json 172.16.0.2 100 100 > robot_state.jsonl

# Save the robot description returned by the controller.
./build/franka_state --urdf 172.16.0.2 > robot.urdf
```

The default host is `172.16.0.2`. The default sample count is 1 and the default
interval is 100 ms. Sampling is best effort. Use `state.time` for the robot's
sample timestamp, in milliseconds with a robot-defined origin.

The original joint-position reader is still available as `./build/joint_positions`. It
reads one snapshot from `172.16.0.2`.

## Output

The default console view groups joint state, dynamics, and frame calculations
into sections and displays matrices by row. Use `--json` to capture the full
serialized robot state and model data as one JSON object per line.

| Key | Contents |
| --- | --- |
| `server_version` | FCI server version reported by the robot |
| `state` | libfranka's serialized RobotState, including measured and desired joint position and velocity, measured joint torque, Cartesian transforms, estimated external wrench and joint torques, contact and collision flags, robot mode, errors, and configured end-effector and payload properties |
| `dynamics` | Joint-space mass matrix (7 × 7), Coriolis/centrifugal torque vector (7), and gravity torque vector (7) |
| `frames` | Model pose (4 × 4), base-frame Jacobian (6 × 7), and body-frame Jacobian (6 × 7) for each of `joint1` through `joint7`, plus `flange`, `end_effector`, and `stiffness` |

Model values in a record use that record's measured joint state and configured
tool and payload parameters. Matrices are flat column-major arrays. The first
three Jacobian rows are linear and the last three are angular. For example, use
`np.asarray(sample["frames"]["end_effector"]["zero_jacobian"]).reshape(6, 7, order="F")`
in NumPy. Wrench fields are model-based estimates. See the
[RobotState reference](https://frankarobotics.github.io/libfranka/0.18.2/structfranka_1_1RobotState.html)
for units and the meaning of each field.

## Cartesian motion

`cartesian_move` commands arm motion. Supply two reachable and unobstructed
absolute end-effector points in the robot base frame, in meters. Check both
straight segments, from the current pose to A and from A to B, including
clearance for the arm and tool.

```text
./build/cartesian_move AX AY AZ BX BY BZ
```

The command connects to `172.16.0.2`. It holds the starting commanded
end-effector orientation, slows to
zero at A and B, and limits the planned translational speed to 0.05 m/s. It uses
libfranka Cartesian pose control and the controller's existing collision
settings.

## Continuous external force

```bash
./build/external_force 172.16.0.2       # Print approximately 10 readings per second.
./build/external_force 172.16.0.2 50    # Set the display interval to 50 ms.
```

Press Ctrl+C to stop. Each sample shows the robot timestamp, force magnitude,
and two wrench readings. **Base / K** expresses the force and torque in robot
base axes, with the moment taken about the stiffness frame origin. **End
effector / EE** expresses them in end-effector axes and shifts the moment to the
end-effector origin using the configured stiffness-frame offset. Force is in
newtons and torque is in newton-meters. The program reverses Franka's wrench
sign convention to report the wrench acting on the tool. Franka can report
zero near a singularity. Each sample also prints the seven filtered external
joint torque estimates (`tau_ext_hat_filtered`) in N·m, using Franka's
joint-space signs. See the
[libfranka RobotState reference](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html).

### Real-time scheduling

`cartesian_move` uses `franka::RealtimeConfig::kIgnore`, so failure to acquire
real-time priority does not abort the motion command. Control-loop timing still
depends on the workstation and network.

When using libfranka's enforced real-time mode, if libfranka reports `unable to set realtime scheduling: Operation not permitted`,
run `ulimit -r` in the terminal that launches `cartesian_move`. A result of
`0` means that session lacks real-time priority permission. Franka documents
how to boot a PREEMPT_RT kernel and grant the robot operator's account
real-time scheduling permissions. Start a fresh login session after changing
group membership or limits, then check `ulimit -r` again.
[Franka real-time workstation setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)

## Joint motion

`joint_move` connects to `172.16.0.2` and moves the FR3 from its current
commanded joint configuration to one destination. Enter seven absolute angles
in joint order, in radians. Use `./build/joint_positions` to read the current
angles when choosing a destination. Check clearance for the entire arm and
tool along the path.

```text
./build/joint_move Q1 Q2 Q3 Q4 Q5 Q6 Q7
```

The program validates the destination against the FR3 joint position limits and uses
smooth joint-space interpolation with a planned peak of 0.15 rad/s per joint.
It enables libfranka's joint command rate limiter and uses the same
`RealtimeConfig::kIgnore` setting as `cartesian_move`.
[Franka joint limits](https://frankarobotics.github.io/docs/robot_specifications.html)

## Computed torque experiment

```bash
./build/computed_torque
```

This command drives the FR3 at `172.16.0.2` and requires real-time scheduling
by default. To allow startup when real-time scheduling is unavailable, use
`./build/computed_torque --ignore-realtime`. The torque loop can
still miss deadlines and abort on communication faults.
Joints 2 and 4 follow in-phase 0.2 Hz sine references centered on their
measured starting angles, each with a 15° peak amplitude (30° peak-to-peak).
The 15-second reference includes a 2.5-second ramp at each end,
followed by a one-second hold. The other joints use PD torque feedback to
hold their initial positions. Check arm and tool clearance for the full
swing. Press Ctrl+C to request an early stop and save the samples.

For joints 2 and 4, the commanded acceleration is the analytic reference
acceleration plus position and velocity feedback. Edit
`config/computed_torque.conf` to tune the gains independently. The controller
reads it at startup, so gain changes take effect on the next run without
rebuilding. The configured gains are 20 s⁻² and 9 s⁻¹ for joint 2, and 25 s⁻²
and 10 s⁻¹ for joint 4.
The controller multiplies that vector by the corresponding 2 × 2 block of
the mass matrix and adds the corresponding Coriolis terms. Franka supplies
gravity and friction compensation. The other joints use the PD gains from
the [libfranka 0.18.2 external joint impedance example](https://raw.githubusercontent.com/frankarobotics/libfranka/0.18.2/examples/joint_impedance_control.cpp).
The controller applies torque and slew-rate limits and stops on excessive
tracking or hold error.

The program buffers one sample per callback and writes the CSV after the
loop. The CSV contains robot time, elapsed time, measured position and
velocity, reference position, velocity and acceleration, measured torque,
the previous desired torque reported by Franka, and both raw and limited
torque requests. New logs also include the estimated external force acting at
the stiffness frame, expressed in robot base axes, and its magnitude, all in
newtons. The force sign matches the `external_force` program. The logged limited request precedes any additional
libfranka filtering. Runs are numbered `001.csv`, `002.csv`, and so on in
`data/computed_torque/`, relative to the project root. If libfranka supplies a fault log through
`ControlException::log`, it is exported separately as
`<run-number>.franka.csv` with `franka::logToCSV`.


### Plotting a run

Run the plotting script with a Python environment that has Matplotlib installed.
The first command opens the most recent automatically saved run. Pass a CSV path
to select an earlier run, and use `--output` to save the figure.

```bash
python3 plot/plot_computed_torque.py
python3 plot/plot_computed_torque.py data/computed_torque/002.csv --output run02.png
```

The plots compare measured and reference position and velocity for joints 2 and
4, show measured and requested torque, and show position error and requested
torque for the five holding joints.

For logs with force data, the figure includes force components and magnitude.

## Cartesian computed torque

`cartesian_computed_torque` drives the end effector in base-frame x and z
using joints 2 and 4. The other five joints use PD control to hold their
initial angles. End-effector y and orientation are unconstrained and can
change with joint geometry and hold error.

```bash
source scripts/franka_env.sh
./build/cartesian_computed_torque 0.01 -0.01
```

The arguments are relative x and z offsets in meters, with a combined
magnitude up to 0.05 m. The quintic reference takes 20 seconds each way and
holds the starting x/z target for another second. Check clearance along the
move, including any orientation change. The controller runs with libfranka's
real-time scheduling requirement disabled and stops near an x/z singularity,
near an active joint limit, or when joint excursion or tracking error exceeds
its limit.

Use `./build/cartesian_computed_torque 0 0` to hold the starting x/z target
for six seconds and log drift. For a nonzero x offset and zero z offset,
the controller holds the starting x/z target for one second before starting
the x trajectory. Both modes stop if measured z deviates by more than 5 mm.

The x and z tracking gains are configured independently in
`config/cartesian_computed_torque.conf`. Both axes default to $K_p=100$ s⁻²
and $K_d=20$ s⁻¹. The controller reads the file at startup.

The controller extracts the active mass block $M_a$ and Coriolis vector
$c_a$ from the libfranka model. With $J$ the x/z Jacobian for joints 2 and 4,
it computes $\Lambda = J^{-T} M_a J^{-1}$ and
$\mu = J^{-T}c_a - \Lambda\dot J_{xz}\dot q$. The velocity and Jacobian
derivative terms use all seven measured joint velocities. The dynamics
assume zero acceleration on the held joints. The wrench is
$F = \Lambda a_{cmd} + \mu$, and the active torque command is $J^T F$.
The robot handles gravity compensation through its torque interface.

CSV logs in `data/cartesian_computed_torque/` include measured and reference
x/z motion, Cartesian inertia and bias, requested wrench, joint positions,
torque commands after magnitude and rate limiting, and estimated external
force along the three base-frame axes.

The CSV also records modeled gravity torque and the sum of the limited
command and modeled gravity for each joint. The plotter shows this sum for
joints 2 and 4 when available. The estimate excludes the robot's internal
friction compensation.

New Cartesian logs use numbered filenames such as `001.csv`. To plot
the latest run or save a specific run as an image, set up a Python virtual
environment with Matplotlib. Run the first and third commands once, and
activate the environment in each shell where you use the plotters.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install matplotlib

python3 plot/plot_cartesian_computed_torque.py
python3 plot/plot_cartesian_computed_torque.py data/cartesian_computed_torque/001.csv --output cartesian_run.png
```

## Cartesian z force control

`cartesian_force_control` regulates downward force against a horizontal
surface using joints 2 and 4. Place the surface just below the tool at the
preset approach pose. The other joints
hold their starting angles with PD feedback. The controller regulates
force only, so x is monitored for drift.

```bash
source scripts/franka_env.sh
./build/cartesian_force_control 2.0
```

Enter `h` to return to the seven joint angles captured when the program
started. This works before `t` and after a force run returns to the prompt.
Check clearance along the return path.

Press `t` to move to the preset seven-joint configuration. Check clearance
along the approach before pressing `t`. Once the arm reaches the pose, press
`c` to start force control immediately, including when the tool is still
above the surface. The arm must be within 0.02 rad of each target joint
angle when starting. The controller stops if the estimated upward reaction
has not reached 0.5 N for 30 ms within 4 s or 10 mm of vertical travel.
Enter `q` to exit while waiting. Submit each command by pressing Enter.

The positive argument is the desired compression in newtons. The reference
ramps from the initial estimated compression to the target over two seconds,
holds for two seconds, then ramps back to the initial reference. Keep the tool
clear of contact when `t` settles. The controller captures the estimated
external wrench and all seven external joint torques at that pose and subtracts
the captured values during the `c` run. Force feedback uses the corrected
base-frame z component, with upward reaction on the arm positive. The CSV also
records the captured z-force and joint-torque offsets.
The controller sends the negative of the feedforward plus PI force through
the z row of the Jacobian transpose for joints 2 and 4. Franka handles
gravity and friction compensation.

Set force-only contact parameters in `config/cartesian_contact.conf`.
This includes the robot address, real-time scheduling policy, preset pose,
joint move timing, hold gains, torque and torque-rate limits, joint
excursion limits, and contact and workspace limits. For excursion limits,
edit `active_excursion_rad` for joints 2 and 4 and `held_excursion_rad`
for the other joints. The values are relative to the pose at `c`.

Set the force-only gains, run duration, and contact-loss threshold in
`config/cartesian_force_control.conf`. Settings are read at program startup.
The controller bounds the force and integral terms, limits joint torque and
torque rate, and stops on contact loss, excess force, or excessive drift.
Numbered CSVs under `data/cartesian_force_control/` log the requested target
(`force_target_N`), ramped reference, measured,
and commanded force, x/z position, commanded joint torques, and whether
contact has been confirmed. The integrator starts accumulating after contact
is confirmed to avoid winding up in free space.

Only the run started with `c` is recorded. The plotter uses the requested
target from the CSV. For older logs, pass `--target` with the original request
or omit it to show the recorded ramped reference.

```bash
source .venv/bin/activate
python plot/plot_cartesian_force_control.py
python plot/plot_cartesian_force_control.py data/cartesian_force_control/002.csv --target 2 --output force_run.png
```

## Hybrid Cartesian x motion and z force

```bash
source scripts/franka_env.sh
./build/cartesian_hybrid_control 2.0 0.01
./build/cartesian_hybrid_control 2.0 0.20
```

The first command requests 2 N of downward compression and a 1 cm move
along positive base-frame x. The second requests 20 cm. Set the duration
for either move with `x_motion_s` in `config/cartesian_hybrid_control.conf`.
The program checks the peak reference speed against the configured limit.
The x displacement is signed and limited to
30 cm in magnitude by default. The `t`, `c`, `h`, and `q` commands use the same
workflow as Cartesian force control. Keep the tool clear of the surface
when `t` settles so the estimated external wrench and joint torque offsets
can be captured in free space.

Entering `c` starts the two-second z-force ramp and the one-way quintic
x trajectory together. The x move takes 40 seconds with the supplied
configuration. The controller stops if it reaches the no-contact
time or travel limit. After the x move, it holds the endpoint for one
second and then ramps down the force. The x integral is active during the
final hold, with bounded acceleration and anti-windup.

The active-joint command is $\tau_{2,4}=J_{xz}^T[F_x,F_z]^T$.
The x wrench is $F_x=\Lambda_{xx}a_x^*+\mu_x$. The z wrench comes entirely
from force feedforward plus PI with offset-corrected force feedback,
command saturation, and integral anti-windup.
Franka handles gravity and friction compensation. The other five
joints hold their starting angles.

The hybrid controller reads all settings from
`config/cartesian_hybrid_control.conf`, including its robot address, preset
pose, hold gains, joint excursion and torque limits, contact settings,
hybrid gains, and x trajectory timing.
In the hybrid config, `active_excursion_rad` applies to joint 2 and
`joint_4_excursion_rad` applies to joint 4.
Numbered logs in
`data/cartesian_hybrid_control/` include the requested and corrected
measured z force, x reference and measured motion, the PI and summed wrench
commands, and the limited joint torques. The force plotter accepts a hybrid
CSV path directly.

To plot x tracking, z-force tracking, wrench contributions, and active joint
torques from the latest hybrid run or save a specific run as an image, use

```bash
source .venv/bin/activate
python plot/plot_cartesian_hybrid_control.py
python plot/plot_cartesian_hybrid_control.py data/cartesian_hybrid_control/002.csv --output hybrid_run.png
```

## IAC1 adaptive effective normal

```bash
source scripts/franka_env.sh
./build/cartesian_iac1_control 2.0
```

Use `t` to move to the preset pose and tare the wrench with the tool clear
of the surface. Enter `c` to ramp a contact-seeking force along the nominal
normal within the configured travel and time bounds. Once contact is confirmed,
IAC1 runs for `run_duration_s` with constant desired compression. The other
five joints hold their starting angles. Enter `h` at the prompt to return
to the startup pose.

Set all parameters in `config/cartesian_iac1_control.conf`. The target is a
fixed Cartesian point. The default is the first confirmed contact point.
Use `target_x_offset_m` and `target_z_offset_m` to shift that point along the
surface. The normal points into free space and is parameterized as
$[-\sin\theta,\cos\theta]^T$ in base-frame x/z. Use a low-friction surface
because the offset-corrected x/z force magnitude is used as the scalar
contact-force estimate and includes any tangential force.

The wrench uses full reduced Cartesian inverse dynamics and the constant
desired force along the effective normal. The estimator integrates the
IAC1 angle state using the actual sample period and a centered directional
difference of the libfranka model for $\dot\Lambda$. Angle integration is
paused when a user torque limit intervenes. Franka supplies gravity and
friction compensation. The configured angle bound is a runtime stop
threshold. A convergence sector for the FR3 has yet to be verified.

Numbered CSVs in `data/cartesian_iac1_control/` include measured and desired
x/z, the scalar contact-force estimate, the adaptive state and effective
angle, the wrench and the limited joint torques.

Plot the latest IAC1 run or export a specific run as an image.

```bash
source .venv/bin/activate
python plot/plot_cartesian_iac1_control.py
python plot/plot_cartesian_iac1_control.py data/cartesian_iac1_control/001.csv --output iac1_run.png
```

For a flat surface, the configured `initial_effective_angle_rad` starts
adaptation with a +10° error. To plot the signed effective-normal error
after contact, pass `--surface-angle-deg 0` for a flat surface or enter the
known signed surface tilt from +x toward +z. This angle is used only for the
plot. The controller reads the initial effective angle from its config at
startup.
