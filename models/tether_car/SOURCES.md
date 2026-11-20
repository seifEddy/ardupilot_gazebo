# Model sources

The car definition is adapted from Open Robotics' **Prius on Sonoma Raceway** world, version 1, and **Prius Hybrid** model, version 2. The Prius model metadata credits Ian Chen (`iche033@osrfoundation.org`).

- [Source world](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Prius%20on%20Sonoma%20Raceway)
- [Source Prius model](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius%20Hybrid)

The `meshes/` and `materials/` assets are copied from the source Prius model. The body, windows, wheel visuals, wheel dimensions and inertial properties are retained. The local definition uses a single body collision, rigid rear-wheel mounts, front steering, and a forward +X model axis. Interior visuals, vehicle cameras, the steering-wheel mechanism and rear suspension are omitted. The winch links and drum joint are copied from this repository's `tether_station`, with its ground anchor replaced by a fixed roof mount.
