# rover_sensors_bringup

Starts the Rover A1 sensor payload. `rover-a1-sensors` (`rover_docker`) runs it in its
`start.sh`.

```bash
ros2 launch rover_sensors_bringup rover_sensors.launch.py
```

| Argument | Default | Starts |
|----------|---------|--------|
| `use_gps` | `$ROVER_USE_GPS`, else `false` | `rover_gps` (`rover_gps_driver` + `rover_gps_node`) |
| `use_lidar` | `$ROVER_USE_LIDAR`, else `false` | `rover_rs16_lidar` (`rover_rs16_lidar_node`) |
| `namespace` | `$ROVER_NAMESPACE`, else empty | passed to every driver |
| `common_dir_path` | empty | passed to every driver |
| `log_level` | `INFO` | passed to every driver |

See the repo `README.md` for the topic contract.
