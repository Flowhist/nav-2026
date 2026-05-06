# DM IMU Vendor Modules

This directory holds the low-level Damiao IMU serial protocol parser used by
the finav ROS wrapper in `scripts/imu/dm_imu_publisher.py`.

The ROS-facing node and one-time configuration tool stay under `scripts/imu`;
only reusable device protocol modules live here.
