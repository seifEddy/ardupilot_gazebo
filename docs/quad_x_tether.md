# Quad X tether simulation

Start the `quad_x_tether` world through the workspace launcher.

```bash
./ardupilot_ws/start_copter_sim.sh gui quad_x_tether
# Or, after sourcing the ROS 2 overlay
ros2 launch robot_description copter_sim.launch.py world:=quad_x_tether mode:=gui
```

The world loads the existing Quad X with its ArduPilot JSON SITL plugin and adds the tether station and `TetherPlugin`. The station sits 1.4 m to the side of the pad. The drum starts at the world's `initial_length`. Automatic payout is available when this value is below `max_length`. A GUI marker represents the cable in cyan when slack and orange under tension. Slack sag is a drawing approximation. The physical model is a massless, compliant line between two endpoints. Cable weight, aerodynamic drag, contact, entanglement, and failure are outside this model.

## Station and joints

- `anchor_joint` is a fixed joint from `world` to the station chassis. It reacts the cable load into the ground.
- `drum_joint` is a one-axis revolute joint between the chassis and drum about station-local +Y. The effective winding radius is 0.08 m. Positive rotation pays line out. Travel is limited in metres by the controller, allowing the drum to rotate several turns.
- The fairlead is a fixed point on the chassis 0.32 m above its base origin. The virtual cable can leave it in any direction. On the aircraft it attaches at `(-0.08, 0, -0.06)` m in the Quad X base-link frame, aft of the center and below the fuselage.

The winch starts in automatic braked payout. At rest, up to the breakaway torque, the brake balances the pull and holds the drum. Above the threshold the cable turns the drum in payout; the brake resists it with Coulomb torque plus viscous damping. The Quad X world uses a breakaway tension of 1.5 N, equivalent to 0.12 N·m at this drum, and a brake damping of 0.02 N·m·s/rad. A speed governor adds resistance over the last 10% of the maximum speed, and Gazebo enforces the joint speed and travel limits. Automatic mode only pays out. It holds the new length when the pull falls.

An explicit length command temporarily runs the motor. The command is clamped to the configured length range and the motor requests six times the length error, limited to ±0.65 m/s of line speed. The winch returns to automatic braked payout once the requested length is reached and the drum has slowed down. It can pay out again if the cable is still pulling above the threshold.

Gazebo provides the drum angle and rate in both modes. The cable model uses those actual values, and the cable applies a payout torque to the drum and the opposite torque to the chassis. The motor is an ideal velocity actuator during a length move. Motor current and gearbox losses are outside the model.

## Dynamics and measurements

The plugin runs during Gazebo's physics update. The tether world uses a 1 ms maximum physics step. It publishes ideal samples at 20 Hz of simulation time.

Let `a` be the fairlead point, `b` the aircraft attachment point, and `u = (b − a) / d`, where `d = |b − a|`. Each point is its link origin plus its body-fixed offset rotated into world coordinates. Gazebo reports the linear velocity at each offset, including the contribution from rotation of the link. Let those velocities be `v_a` and `v_b`.

```text
L       = L_initial + R × drum_angle
L_dot   = R × drum_angular_velocity
x       = max(0, d − L)
x_dot   = u · (v_b − v_a) − L_dot       while stretched
T       = 0                              if d ≤ L
T       = clamp(k × x + c × x_dot, 0, 15) if d > L
F_base  = +T × u
F_drone = −T × u
τ_drum  = R × T
```

The spring stiffness and axial damping come from `stiffness` and `damping` in the world file. Axial damping is separate from the drum brake damping. The damper opposes changing extension, so paying out reduces tension and reeling in increases it when taut. The clamp prevents a compressive force and limits the simulated line load to 15 N. The limit saturates the force and does not disconnect the tether. Forces act at the two attachment points, so a pull away from the aircraft's center of mass also produces a moment.

Gazebo Transport publishes the following under `/tether/`. With the tether bridge running, the same names appear under `/gz/tether/` in ROS 2.

- `length` is paid-out line in metres from the reel encoder.
- `range` is straight-line distance between attachments in metres.
- `extension` is positive stretch in metres.
- `payout_rate` is measured line speed in m/s. Positive means payout.
- `tension` is the ideal load-cell magnitude in newtons, calculated from the force law.
- `anchor_force` is the three-component fairlead force in Gazebo world ENU, in newtons.
- `azimuth` is `atan2(Δy, Δx)` in radians from world +X toward +Y. `elevation` is `atan2(Δz, hypot(Δx, Δy))`, positive above the horizontal. Both describe the line from the fairlead to the aircraft.
- `drum_load_torque` is the line-side torque `R × T` in N·m.

The scalar samples use `gz.msgs.Double` and have no per-message timestamp. The clock bridge supplies simulation time. There is no sensor noise or filtering in these ideal measurements, and they are independent of MAVLink telemetry.

## Set line length and monitor tension

For long-line configurations, reserve some travel above the initial payout. To see automatic release near the pad, set `initial_length` to 3.0 m, keep `max_length` above it, and use an axial `damping` of 3.5 N·s/m for the short-line setup.

Use either Gazebo Transport or ROS 2 to request a 2.5 m payout. The automatic brake resumes after the move. Reeling while the line is taut adds a load to the flight controller.

```bash
gz topic -t /tether/cmd_length -m gz.msgs.Double -p 'data: 2.5'
gz topic -e -t /tether/tension

ros2 topic pub --once /gz/tether/cmd_length std_msgs/msg/Float64 '{data: 2.5}'
ros2 topic echo /gz/tether/tension
```

The world file contains the offsets, length and speed limits, spring stiffness, damping and maximum force. Resetting the world restores the initial line length and automatic brake mode.
