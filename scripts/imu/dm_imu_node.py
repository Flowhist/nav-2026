#!/usr/bin/env python3
"""
DM-IMU ROS2节点 - 整合到uninav包
发布完整6轴IMU数据到 /imu/data
"""

import sys
import os

# 添加dm_imu模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "dm_imu_modules"))

from dm_imu_modules.node import main

if __name__ == "__main__":
    main()
