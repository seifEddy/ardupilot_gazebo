# Mobile tether station on the Prius

`quad_x_tether_car.sdf` loads the Quad X with its existing ArduPilot JSON SITL configuration and a local copy of the Prius from the Open Robotics Sonoma Raceway example. The world uses the existing test field. The car starts at `(0, -2, 0.03)` m and the aircraft starts on the adjacent launch pad.

## Car and roof-mounted station

The local `models/tether_car` directory contains copied OBJ, material, and texture assets, the model definition, and source attribution. The car retains its body and wheel visuals, wheel masses, dimensions, and front-wheel steering. It uses a single body collision and rigid rear-wheel mounts. Vehicle cameras, interior visuals, the steering-wheel mechanism, and rear suspension are omitted. Model +X points forward and +Y points left.

The station links and drum joint are copied from `tether_station`. In the mobile model, `station_mount_joint` fixes the station base to the car chassis. The station base sits 1.5 m above the car model origin, with the drum at 1.687 m. The original fairlead offset places the cable attachment at `(0, 0, 1.82)` m in the car frame. This fixed mount transfers cable forces and torques to the vehicle. The drum still rotates independently about its revolute joint.

The tether world configures `station_model` as `tether_car`, so the existing tether plugin resolves the mobile `base_link`, `drum`, and `drum_joint`. It reads the anchor's world pose and velocity at the fairlead offset every physics update. The cable force therefore responds to both car and aircraft motion, including rotation of either attachment. The [tether force and reel model](quad_x_tether.md) supplies the same automatic take-up, braked payout, and explicit length-command behavior.

## Drive and state plugins

The Gazebo `AckermannSteering` system drives the rear wheels and steers the front wheels. The geometry uses a 2.86 m wheelbase, 1.572 m rear wheel separation, 1.52 m front kingpin width, and 0.31265 m wheel radius. Commands are limited to ±2 m/s and ±1 m/s². Steering is limited to 0.5 rad. A car needs forward or reverse movement to turn.

The model publishes wheel odometry using `AckermannSteering`, 3D simulation ground truth using `OdometryPublisher`, and wheel, steering, mount, and drum joint states using `JointStatePublisher`. The world ground-truth frame is `world`. Wheel odometry uses `car/odom`, whose origin is initialized independently of the car's world spawn offset. Both odometry messages identify the child frame as `car/chassis`.

Gazebo topics are `/model/tether_car/cmd_vel`, `/model/tether_car/odometry`, `/model/tether_car/ground_truth`, and `/model/tether_car/joint_state`. The mobile ROS bridge profile maps them to `/car/cmd_vel`, `/car/odom`, `/car/ground_truth`, and `/car/joint_states`. Commands flow ROS to Gazebo and state flows Gazebo to ROS.

## Run and teleoperate

Build and source the ROS package from the repository root.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
ros2 launch robot_description copter_sim.launch.py world:=quad_x_tether_car mode:=gui
```

In another sourced interactive terminal, start the keyboard node.

```bash
ros2 run robot_description car_teleop.py
```

If your ROS CLI lacks the `run` extension, install `ros-humble-ros2run` or execute the ROS node directly after sourcing the workspace.

```bash
python3 ~/ardupilot-ros2-dev/ros2_ws/src/robot_description/scripts/car_teleop.py
```

- `W` and `S` drive forward and backward.
- `A` and `D` drive forward while turning left and right.
- `Z` and `C` drive backward while turning the front wheels left and right.
- Space commands a stop. `Q` and Ctrl+C exit with a zero command.

The node publishes at approximately 20 Hz and restores the terminal settings when it exits. Releasing the keys sends zero commands after a 0.6 s timeout. Terminal key-repeat settings affect continuous key input. The car obeys its acceleration limit while stopping. Its drive plugin retains the last received command if the keyboard process ends without sending zero. Initial speed is 0.5 m/s and turn rate is 0.2 rad/s. Set the startup parameters for a slower tracking experiment.

```bash
ros2 run robot_description car_teleop.py --ros-args \
  -p speed:=0.25 -p turn_rate:=0.05 -p key_timeout:=0.6
```

Inspect state in additional sourced terminals.

```bash
ros2 topic echo /car/odom
ros2 topic echo /car/ground_truth
ros2 topic echo /car/joint_states
ros2 topic echo /gz/tether/tension
```

For an existing Gazebo instance, select the same profile when starting the ROS bridge.

```bash
ros2 launch robot_description gazebo_bridge.launch.py world:=quad_x_tether_car
```

The standalone launcher accepts `./ardupilot_ws/start_copter_sim.sh gui quad_x_tether_car`. For direct Gazebo startup, set the model and plugin paths as described in the ArduPilot workspace guide and open `quad_x_tether_car.sdf`.

## Aircraft tracking

Start the existing controller after the mobile stack is ready.

```bash
ros2 run robot_description tethered_drone_node --ros-args \
  -p use_sim_time:=true -p target_altitude_z:=15.0
```

The controller's range-and-angle conversion measures the aircraft attachment relative to the moving fairlead, expressed along world ENU axes. Its zero X and Y targets therefore move with the car. `/gz/tether/relative_position` publishes the measured relative vector. For mobile world-coordinate comparisons, calculate the anchor from `/car/ground_truth` by rotating `(0, 0, 1.82)` into world coordinates and adding the car position. Account for the aircraft attachment offset when comparing against MAVROS body position.

The current hardcoded proportional gains are `Kp_xy=0.8` and `Kp_z=1.2` s⁻¹. With proportional feedback alone, a moving car can produce tracking lag. The controller also uses its current length-based takeoff gate and hardcoded 10 m above-home takeoff request. Car velocity feed-forward, tension feedback, velocity-command saturation, tether sample-age checks, and service-response sequencing are further development steps. The mobile world provides the measurements and commands for that development.

## Altitude change and visualization

The controller starts with a default target of 15.0 m above the fairlead. After it approaches the target, change the parameter to 6.0 m.

```bash
ros2 param set /winch_supervisor target_altitude_z 6.0
```

Plot the `x` and `y` fields of `/gz/tether/relative_position` on one PlotJuggler time-series panel and its `z` field on another. The parent workspace's [ROS 2 guide](../../../../../ros2_ws/README.md#run-the-15-m-to-6-m-altitude-demo) gives the complete simulation, aircraft controller, keyboard, and PlotJuggler startup sequence. The [mobile demo GIF](../../../../../README.md#mobile-car-tracking-and-altitude-step) accompanies that sequence.

## Sources

- [Local model attribution](../models/tether_car/SOURCES.md)
- [Gazebo Harmonic AckermannSteering](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html)
- [Gazebo Harmonic OdometryPublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
