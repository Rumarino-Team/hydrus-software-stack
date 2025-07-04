#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["mission_planner"],
    package_dir={"": "src"},
    requires=["std_msgs", "rospy", "geometry_msgs", "actionlib"],
)

setup(**setup_args)
