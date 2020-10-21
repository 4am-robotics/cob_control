#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['twist_controller_config', 'data_collection'],
   package_dir={'': 'src'}
)

setup(**d)
