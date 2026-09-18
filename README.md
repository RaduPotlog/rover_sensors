# rover_sensors

The sensor payload of Rover A1: drivers for the sensors that differ between rovers and missions
(GNSS receiver, lidar, ...). It runs in its own container, `rover-a1-sensors` (`rover_docker`),
separate from the platform (`rover_ros`, container `rover-a1-platform`) and from autonomy
(`rover_orchestrator`, container `rover-a1-orchestrator`).

Swapping or upgrading a sensor changes only this repo and its container. The platform and Nav 2
do not depend on any package here. They depend only on the topic contract below.

## Topic contract

Payload drivers **only publish**. They never subscribe to platform or autonomy topics, so the
payload can restart, or be replaced, without touching the rest of the rover.

| Topic (in `<namespace>`) | Type | Frame | Consumers |
|--------------------------|------|-------|-----------|
| `gps/fix` | `sensor_msgs/NavSatFix` | `<ns>/gps_link` | `rover_localization` (`rover_gps_heading_node`, `rover_navsat_transform_node`) |
| `gps/vel`, `gps/time_reference`, `gps/heading` (HDT only) | | `<ns>/gps_link` | optional |
| `scan` | `sensor_msgs/LaserScan` | `<ns>/lidar_link` | Nav 2 costmaps |
| `rslidar_points` | `sensor_msgs/PointCloud2` | `<ns>/lidar_link` | Nav 2 costmaps via `pointcloud_crop_box` |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | | `rover_diag_manager` aggregator (`/Rover/GPS`, `/Rover/Lidar`), `IsLidarHealthy` BT condition, mission manager |

The diagnostic status names are part of the contract: `rover_gps_node: GPS fix` and
`rover_rs16_lidar_node: Lidar status`.

**TF is not part of this repo.** The drivers only stamp `<ns>/gps_link` / `<ns>/lidar_link` as
`frame_id` and never publish a transform. Both frames are defined by the platform URDF
(`rover_ros/rover_description`) and published by the platform's `robot_state_publisher`. Their
mount poses come from the balena variables `ROVER_{GPS,LIDAR}_LOCALIZATION_{X,Y,Z}` /
`ROVER_{GPS,LIDAR}_ORIENTATION_{R,P,Y}`, which **rover-a1-platform** reads
(`rover_description/launch/rover_load_urdf.launch.py`). Nothing in `rover_sensors` reads them.
They appear on the `rover-a1-sensors` service only because `docker-compose.yml` carries every
`ROVER_*` variable on every service. Moving a sensor needs no rebuild, only new variable values
(which restart the platform container).

## Packages

| Package | Contents |
|---------|----------|
| `rover_gps` | RUTX11 NMEA-over-UDP driver (`rover_gps_driver`, UDP 10110) and GPS fix health (`rover_gps_node`). |
| `rover_rs16_lidar` | RoboSense RS16 driver (UDP 6699/7788, vendored `rs_driver`), LaserScan projection and lidar health in one node. |
| `rover_sensors_bringup` | `rover_sensors.launch.py`: starts the drivers selected by environment variables. |

The GNSS heading alignment that GPS localization needs (`gps/heading_imu`) is not here. It needs
`odom`, which makes it localization logic, so it lives in `rover_ros/rover_gps_heading`.

## Launch

```bash
ros2 launch rover_sensors_bringup rover_sensors.launch.py namespace:=rover use_lidar:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace and TF prefix. |
| `use_gps` | `$ROVER_USE_SENSOR_GPS`, else `true` | Start the GNSS driver. It is on by default, so GPS health stays visible even when localization does not fuse GPS. |
| `use_lidar` | `$ROVER_USE_LIDAR`, else `false` | Start the RS16 driver. Leave it off on rovers with no lidar fitted. |
| `common_dir_path` | empty | Directory with per-package config overrides (`<dir>/<package>/config/...`). |
| `log_level` | `INFO` | Logging level. |

Booleans accept `true`/`1`/`yes`/`on` in any case, because the values usually come straight from
balena variables. `ROVER_USE_GPS` is **not** read here: it selects whether the platform's
localization fuses GPS.

## Adding a sensor

1. Add a driver package that publishes the standard message type for its sensor, in a
   `<ns>/<sensor>_link` frame, plus a `diagnostics` task named `<node>: <status>`.
2. Include its launch file in `rover_sensors_bringup/launch/rover_sensors.launch.py` behind a
   `ROVER_USE_<SENSOR>` flag, and add it to `rover_sensors_bringup/package.xml`.
3. Add its diagnostic prefix to `rover_ros/rover_diag_manager/config/diagnostic_aggregator.yaml`.
4. If it needs a new frame, add the link to `rover_ros/rover_description` once, positioned by
   new `ROVER_<SENSOR>_LOCALIZATION_*` / `ROVER_<SENSOR>_ORIENTATION_*` variables read in
   `rover_description/launch/rover_load_urdf.launch.py`, following the GPS and lidar pattern.
   Payload drivers never publish TF themselves.
5. Third-party sources go in `sensors_deps.repos`, apt dependencies in `package.xml` (rosdep).

## Build and test

```bash
vcs import src < src/rover_sensors/sensors_deps.repos
rosdep install --from-paths src/rover_sensors -y -i
colcon build --packages-up-to rover_sensors_bringup
colcon test --packages-select rover_gps rover_rs16_lidar --parallel-workers 1
```
