#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['lifecycle_test_library'],
    package_dir={'': 'src'},
    scritpts=['scripts/lifecycle_node_test.py']
    )

setup(**setup_args)