#!/usr/bin/env python3
from setuptools import setup

package_name = 'ROS_SplinePathMotionProfiling'
setup(
    name = package_name,
    version = '0.0.0',
    packages = [package_name], 
    package_dir = {package_name: package_name},
    install_requires = ['setuptools']
)