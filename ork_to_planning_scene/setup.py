#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['ork_to_planning_scene']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
