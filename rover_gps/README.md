# rover_gps

GNSS sensor for the Rover A1 payload. The Teltonika RUTX11 router has the GNSS receiver and
forwards its NMEA sentences over UDP. This package parses that stream into `gps/fix` and reports
GPS fix health on `diagnostics`. It runs in the `rover-a1-sensors` container.

The absolute (ENU) heading that GPS localization needs is not computed here: it is localization
logic and lives in `rover_ros/rover_gps_heading`, which consumes this package's `gps/fix`.

## Nodes

| Node | Package / executable | Role |
|------|----------------------|------|
| `rover_gps_driver` | `rover_gps` / `rover_gps_driver_node` | **Lifecycle.** Listens for NMEA on UDP `0.0.0.0:10110` and publishes `gps/fix`. |
| `rover_gps_node` | `rover_gps` / `rover_gps_node` | GPS fix health diagnostics. |

## Interfaces (`rover_gps_node`)

| Direction | Name | Type |
|-----------|------|------|
| sub | `gps/fix` | `sensor_msgs/NavSatFix` (sensor-data QoS, from `rover_gps_driver`) |
| pub | `diagnostics` | hardware id `RoverGps`, task `GPS fix` (`/Rover/GPS` in `diagnostics_agg`) |

The driver also publishes `gps/vel`, `gps/heading` (only if the receiver sends HDT) and
`gps/time_reference`. All four are reliable, depth 10 — reliable rather than sensor-data
best-effort so that reliable subscribers such as `navsat_transform_node` still match.

### The driver

`rover_gps_driver_node` is a C++17 port of `nmea_navsat_driver`'s `nmea_socket_driver`, which it
replaces; the Python package is gone. Parameter and topic names are unchanged, so the config file
and the launch remappings are the same as before.

It is a lifecycle node because it owns the UDP socket: the socket is bound in `on_activate` and
released in `on_deactivate`. The launch file sets `autostart=True`, so it comes up active.

Behaviour matches the Python driver, including two quirks kept deliberately because
`rover_localization` is tuned to them: the altitude covariance carries an extra factor of two
(upstream marks it `FIXME`), and the HDT heading is published as-is, not rotated into ENU.
Four things are fixed rather than reproduced:

- **Multi-sentence datagrams.** Upstream trimmed the whole datagram and split on `\n`, leaving a
  `\r` on every line but the last; its end-anchored regex then rejected those lines. Each line is
  trimmed here, so a datagram carrying GGA + VTG yields both.
- **GST altitude error.** `alt_std_dev` is the last field, so upstream parsed it together with the
  `*XX` checksum and always got NaN, discarding the receiver's altitude estimate. The checksum is
  stripped before the fields are split.
- **Truncated sentences.** Upstream indexed the field list blindly and raised `IndexError`, which
  killed its receive loop. Missing fields become NaN / 0 here.
- **HDT edge cases.** Upstream tested the heading for truthiness, which dropped a valid `0` (due
  north) and let NaN through. The test is on NaN instead.

### `GPS fix` diagnostic

| Level | When |
|-------|------|
| STALE | No fix message received yet, while `gps_required`. |
| ERROR | While `gps_required`: no fix message for `fix_timeout_s` (NMEA link lost), or horizontal std > `error_horizontal_std_m`. |
| WARN | Receiver reports no fix, horizontal std > `warn_horizontal_std_m`, or rate < `expected_rate_hz * min_rate_ratio`. Without `gps_required`, the STALE and ERROR cases above are WARN too, with "(not used for localization)" in the message - e.g. the GPS switched off on the RUTX11. |
| OK | Otherwise. |

Values shown: fix status, latitude/longitude/altitude, horizontal std (from the `NavSatFix`
covariance, which the driver computes from HDOP), rate, age and fix count.

## Parameters (`config/rover_gps.yaml`)

| Name | Default | Description |
|------|---------|-------------|
| `expected_rate_hz` | `1.0` | GGA rate configured on the RUTX11. |
| `min_rate_ratio` | `0.5` | WARN below `expected_rate_hz * min_rate_ratio`. |
| `fix_timeout_s` | `3.0` | ERROR (WARN without `gps_required`) when no fix arrives for this long. |
| `warn_horizontal_std_m` / `error_horizontal_std_m` | `5.0` / `20.0` | Accuracy thresholds [m]. |
| `gps_required` | `true` | `false` caps the diagnostic at WARN: no data, data timeout and accuracy beyond `error_horizontal_std_m`. The launch file sets it from `ROVER_LOCALIZATION_SOURCE`. |

All parameters are read-only; invalid values or `warn > error` stop the node at startup.

Driver parameters (`rover_gps_driver`): `ip` `0.0.0.0`, `port` `10110`, `buffer_size` `4096`,
`timeout_sec` `2`, `frame_id` `gps_link` (prefixed with the namespace through `tf_prefix`),
`time_ref_source` `gps`, `useRMC` `false`, and `epe_quality0/1/2/4/5/9` — the default position
error per GGA fix quality, which sets the fix covariance together with HDOP. `gps_link` is defined in
`rover_description`. Its offset comes from `ROVER_GPS_LOCALIZATION_{X,Y,Z}` and
`ROVER_GPS_ORIENTATION_{R,P,Y}` and defaults to the body origin.

## Launch

```bash
ros2 launch rover_gps rover_gps.launch.py namespace:=rover
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace and TF prefix. |
| `rover_gps_config_path` | `config/rover_gps.yaml` | Parameter file for both nodes. |
| `common_dir_path` | empty | If set, the default config is read from `<common_dir_path>/rover_gps/config/`. |
| `log_level` | `INFO` | Logging level. |
| `gps_required` | `false` if `$ROVER_LOCALIZATION_SOURCE` is `indoor`, `slam` or `amcl`, else `true` | Sets `gps_required`. Nothing localizes on GPS with the lidar sources and the rover is usually indoors, where tens of metres of error are normal and the GPS may be switched off. |

`rover_sensors_bringup` starts it when `ROVER_USE_GPS` is true (default `false`).

## RUTX11 configuration

The RUTX11 (`192.168.1.1`) must forward NMEA over **UDP** to the rover (`192.168.1.201:10110`)
at 1 s intervals. `scripts/rutx11_gps_nmea_forwarding.sh` does this through `uci`:

```bash
cd src/rover_sensors/rover_gps/scripts
./rutx11_gps_nmea_forwarding.sh show                 # read-only; prompts for the router password
RUTX11_PASSWORD=... ./rutx11_gps_nmea_forwarding.sh apply
```

`apply` sets these values, commits them and restarts `gpsd`:

- `gps.gpsd.enabled=1`
- `gps.nmea_forwarding.{enabled=1, proto=udp, hostname=$ROVER_HOST, port=$NMEA_PORT}`
- `gps.{GPGGA,GPRMC,GPVTG}.{forwarding_enabled=1, forwarding_interval=$NMEA_INTERVAL_S}`

It can be overridden with `RUTX11_HOST`, `RUTX11_USER`, `ROVER_HOST`, `NMEA_PORT` and
`NMEA_INTERVAL_S`. In the WebUI the same settings are under
**Services → GPS → NMEA → NMEA forwarding**. The antenna must see the sky: without a fix the
router still forwards GGA, and `GPS fix` reports WARN "No GNSS fix.".

Check that NMEA arrives on the rover. The driver holds the port while it is active, so
deactivate it first — no need to kill it:

```bash
ros2 lifecycle set /rover_gps_driver deactivate
nc -ul 10110
ros2 lifecycle set /rover_gps_driver activate
```

## Layout

```
domain/          GnssFix value type, GpsHealthEvaluator, output ports — no ROS
domain/nmea/     NMEA 0183: checksum, sentence value types, parser (GGA/RMC/VTG/GST/HDT),
                 NmeaFixAssembler (sentences → fix/velocity/heading/time) — no ROS
application/     MonitorGpsUseCase (fix → health report), IngestNmeaUseCase (sentence →
                 checksum → parse → assemble → output port)
infrastructure/  RoverGpsNode (composition root, parameters, subscription),
                 RoverGpsDriverNode (lifecycle, UDP receive thread), UdpNmeaReceiver (POSIX socket),
                 Ros2GpsHealthPublisher / Ros2NmeaPublisher, msg conversions
```

The unit tests in `test/unit/` cover the domain, the use cases and the message mapping without
a ROS graph — including the NMEA checksum, parser and fix assembler, which had no tests at all
while the driver was the vendored Python package.
`test/integration/test_rover_gps_node.cpp` runs the real node in-process and checks the
STALE → OK diagnostics and threshold validation. `test/integration/test_rover_gps_driver_node.cpp` drives the driver through its
lifecycle against a real loopback UDP socket and checks that a datagram becomes a `NavSatFix`
and that deactivating releases the port. Run them with `colcon test --packages-select rover_gps`.
`colcon test` also runs `ament_lint_auto` (uncrustify excluded; see `CPPLINT.cfg` and
`flake8.ini`) and `scripts/check_layer_purity.sh`, which fails if `domain/` or `application/`
includes ROS, socket headers or an outer-layer header.
