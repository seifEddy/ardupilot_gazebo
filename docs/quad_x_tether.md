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

The winch starts in automatic braked payout with gentle slack take-up. Above `initial_length`, a small winding torque pulls excess cable back onto the drum. At rest the brake holds the drum when the cable tension balances the take-up force or exceeds it by up to `breakaway_tension`. Above that threshold the cable turns the drum in payout, resisted by the take-up torque, Coulomb brake torque and viscous damping. Viscous damping also opposes inward rotation. A speed governor adds resistance over the last 10% of the maximum payout speed, and Gazebo enforces the joint speed and travel limits.

The Quad X world preserves 3.0 m of loose line using `initial_length`. It sets `takeup_tension` to 0.25 N, equivalent to a winding torque of 0.02 N·m at the 0.08 m drum radius, and caps automatic inward line speed with `max_retract_speed` at 2.0 m/s. The current breakaway tension is 0.0025 N, brake damping is 0.00098 N·m·s/rad, and maximum payout speed is 2.0 m/s. Automatic take-up stops at the initial length. It becomes active again whenever the drum pays out above that length. A value of zero for `takeup_tension` disables take-up and restores automatic payout with holding. Other worlds that omit the setting retain that behavior.

An explicit length command temporarily runs the motor. The command is clamped to the configured length range and the motor requests six times the length error, limited to ±`max_payout_speed` of line speed. The winch returns to automatic operation once the requested length is reached and the drum has slowed down. It can then pay out under sufficient load or take up excess slack. An explicit command can retract below the 3.0 m automatic take-up floor, down to `min_length`.

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

Automatic take-up applies torque to the drum through `JointForceCmd`. Its winding torque is `R × takeup_tension` whenever `L > L_initial`. For inward rotation the joint torque is `−R × takeup_tension − brake_damping × drum_angular_velocity`. This permits the cable load to balance the take-up torque. At rest, above the initial length, the holding tension lies between `takeup_tension` and `takeup_tension + breakaway_tension`. Inward speed is bounded by both `max_retract_speed` and the remaining distance to the initial length per physics step. The take-up setting specifies the actuator's equivalent line force. Actual tether tension still comes from the spring-damper law and can exceed it during aircraft motion.

Slack cable transmits zero force. As slack is removed, the cable starts stretching and resists the winding torque. With the world's current stiffness of 10.0 N/m and axial damping of 1.0 N·s/m, a stationary 0.25 N tension corresponds to approximately 0.025 m (2.5 cm) of elastic stretch above the paid-out length. Increase `takeup_tension` for stronger recovery, or reduce it for gentler contact. `max_retract_speed` caps the inward speed, while brake damping can make actual recovery slower. These settings also affect the tension required for outward payout. The model retains the original 3.0 m of slack even when the aircraft is closer to the station.

For slack cable, neglecting other losses and travel limits, the approximate steady inward line speed follows from balancing winding torque against viscous damping.

```text
v_retract ≈ min(max_retract_speed, R² × takeup_tension / brake_damping)
          ≈ min(2.0, 0.08² × 0.25 / 0.00098)
          ≈ 1.63 m/s
```

This approximation assumes positive brake damping. Drum inertia determines acceleration toward that speed, and cable tension reduces the available winding torque. Raising only `max_retract_speed` helps when the speed limit is the active constraint. `takeup_tension` sets the winding force, `brake_damping` affects both inward and outward motion, and the cable's `stiffness` and `damping` determine the transmitted tension. `max_tension` sets the cable force saturation.

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

## Station-relative ROS position control

The ROS supervisor reconstructs the aircraft attachment position relative to the fairlead using the endpoint range and the two angles.

```text
x = range × cos(elevation) × cos(azimuth)
y = range × cos(elevation) × sin(azimuth)
z = range × sin(elevation)
```

The vector is expressed along Gazebo world ENU axes. `range` measures endpoint separation, while paid-out `length` also reflects drum motion, slack, and cable stretch. Elevation is the angle above the horizontal plane, so a vertical line has elevation π/2 and zero horizontal projection.

The ROS demo uses this vector for proportional XYZ velocity control, with fixed zero X and Y targets and a runtime `target_altitude_z` parameter. It also publishes relative and translated position diagnostics. The Gazebo reel responds to cable load independently of the ROS controller. Launch, parameter, frame-comparison, and takeoff-sequencing details are in the parent workspace's [ROS 2 guide](../../../../../ros2_ws/README.md#run-the-tether-supervisor).

## Set line length and monitor tension

For long-line configurations, reserve some travel above the initial payout. The current world starts with 3.0 m of paid-out cable, allows up to 500.0 m, and uses axial damping of 1.0 N·s/m. Above the initial length, stationary payout begins when tension exceeds approximately 0.2525 N, the sum of the take-up force and breakaway tension. During payout, viscous braking adds speed-dependent resistance.

Use either Gazebo Transport or ROS 2 to request a 2.5 m payout. Automatic payout and slack take-up resume after the move. Reeling while the line is taut adds a load to the flight controller.

```bash
gz topic -t /tether/cmd_length -m gz.msgs.Double -p 'data: 2.5'
gz topic -e -t /tether/tension

ros2 topic pub --once /gz/tether/cmd_length std_msgs/msg/Float64 '{data: 2.5}'
ros2 topic echo /gz/tether/tension
```

The world file contains the offsets, length and speed limits, spring stiffness, damping and maximum force. Resetting the world restores the initial line length and automatic brake mode.
