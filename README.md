# Franka experiments

`franka_state` reads the arm through the Franka Control Interface (FCI) using
libfranka. The program opens a read-only session to read robot state and query the model.

## Build and run

Use the libfranka release compatible with your robot's system software. The
program was built with libfranka 0.18.2. Enable FCI and use the robot's FCI IP
address or hostname.

On this workstation, add the Pinocchio library directory before building and running.

```bash
export LD_LIBRARY_PATH="/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
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
python3 scripts/plot_computed_torque.py
python3 scripts/plot_computed_torque.py data/computed_torque/002.csv --output run02.png
```

The plots compare measured and reference position and velocity for joints 2 and
4, show measured and requested torque, and show position error and requested
torque for the five holding joints.

For logs with force data, the figure includes force components and magnitude.
