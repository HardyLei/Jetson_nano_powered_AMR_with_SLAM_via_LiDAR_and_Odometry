"""Catch stale YDLIDAR YAML keys that ROS otherwise ignores."""

from pathlib import Path
import re

import yaml


def test_all_lidar_yaml_keys_are_declared():
    """Match each config key to an actual C++ parameter declaration."""
    driver = Path(__file__).resolve().parents[1] / 'ros2_ws/src/ydlidar_ros2_driver-master'
    source = (driver / 'src/ydlidar_ros2_driver_node.cpp').read_text(encoding='utf-8-sig')
    declared = set(re.findall(r'declare_parameter\("([^"]+)",', source))
    for file in (driver / 'params').glob('*.yaml'):
        params = yaml.safe_load(file.read_text())['ydlidar_ros2_driver_node']['ros__parameters']
        assert params and set(params) <= declared, (file, set(params) - declared)
