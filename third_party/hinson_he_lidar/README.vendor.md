# Hinson HE lidar vendor source

This directory contains the driver sources imported from
`hi_driver_ros2_20260226.zip` supplied with the HE-3051 lidar.

- Upstream package version: `hi_ros2 0.0.3`
- Archive SHA-256: `2d414e1b7653ebe670a83fb00b99b77782ba719755a00c033c674de37ce0e2cb`
- Imported components: network transport, HE protocol parser, LaserScan publisher,
  and optional shadow filter
- Not imported: the upstream package manifest, launch files, custom messages,
  and command service

Finav-specific changes keep only the sensor-data path, use SensorDataQoS, make
the driver safe to run as two node instances, and correct first-frame/NTP scan
timestamps. The upstream archive does not contain a license file and declares
its license as TODO; obtain redistribution terms from the manufacturer before
distributing this source outside the project.
