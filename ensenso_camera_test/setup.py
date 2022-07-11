from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["ensenso_camera_test"], package_dir={"": "src"})

setup(**d)
