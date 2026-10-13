from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_args = generate_distutils_setup(packages=['cage_mission_fsm'], package_dir={'': 'src'})
setup(**setup_args)
