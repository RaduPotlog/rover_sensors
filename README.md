# rover_sensors

Mechatronics Academy's Rover A1 sensor payload: drivers for the sensors that differ between
rovers and missions, running in their own container (`rover-a1-sensors`).

- [`rover_gps`](rover_gps/README.md) - RUTX11 NMEA-over-UDP GNSS driver (`rover_gps_driver`,
  lifecycle) and GPS fix health diagnostics (`rover_gps_node`).
- [`rover_rs16_lidar`](rover_rs16_lidar/README.md) - RoboSense RS16 lidar in one lifecycle
  node: point cloud, `scan` projection and stream health, with `rs_driver` vendored.
- [`rover_sensors_bringup`](rover_sensors_bringup/README.md) - `rover_sensors.launch.py`,
  starts the drivers selected by `ROVER_USE_GPS` / `ROVER_USE_LIDAR`.

The GNSS heading alignment that GPS localization needs is not here. It needs `odom`, so it
lives in `rover_ros/rover_gps_heading`.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b master https://github.com/RaduPotlog/rover_sensors.git src/rover_sensors
```

No package here builds against `rover_ros`. The platform is only needed at runtime: its URDF
defines the `<namespace>/gps_link` and `<namespace>/lidar_link` frames the drivers stamp.

### Setup environment variables

```bash
export ROS_DISTRO=lyrical

# Must match the platform, otherwise the drivers publish outside the rover's namespace.
export ROVER_NAMESPACE=rover

# Which drivers to start. Both default to false.
export ROVER_USE_GPS=true
export ROVER_USE_LIDAR=true
```

### Clone dependency

```bash
vcs import src < src/rover_sensors/sensors_deps.repos
```

The list is currently empty: `rs_driver` is vendored in `rover_rs16_lidar/3rdparty` and the
GNSS driver is in-tree.

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_sensors_bringup

source install/setup.bash
```

### Running

#### Real rover:

```bash
ros2 launch rover_sensors_bringup rover_sensors.launch.py use_gps:=true use_lidar:=true
```

#### Simulated rover:

Not needed. `rover_gazebo` publishes the same `<namespace>/scan`, `rslidar_points` and
`gps/fix` from simulated sensors.

### Testing

```bash
colcon test --packages-select rover_gps rover_rs16_lidar --parallel-workers 1
colcon test-result --all
```

Run node tests one at a time: parallel workers make them flaky under the zenoh middleware.

## Related repositories

A complete rover is three repositories, one per container:

- [`rover_ros`](https://github.com/RaduPotlog/rover_ros) - the platform
  (`rover-a1-platform`).
- [`rover_sensors`](https://github.com/RaduPotlog/rover_sensors) - this one, the sensor
  payload, GNSS and lidar drivers (`rover-a1-sensors`).
- [`rover_orchestrator`](https://github.com/RaduPotlog/rover_orchestrator) - the autonomy
  stack, Nav 2 and mission supervision (`rover-a1-orchestrator`).
