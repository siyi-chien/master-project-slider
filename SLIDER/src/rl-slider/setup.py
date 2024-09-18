from setuptools import setup,find_packages
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-gazebo'))
setup(name='rl-slider',        # Secondary directory
    version='0.1',
     packages=find_packages(),
    install_requires=['gym'],   # And any other dependencies foo needs，
    package_data={'gym-gazebo': ['slider/gazebo/launch/*.launch']},
)