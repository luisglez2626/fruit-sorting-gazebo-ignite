# ==============================================================================
# File: setup.py
# Purpose: Python package setup script. It tells ROS 2 how to install the 
#          Python modules and defines the executable entry points (nodes).
# ==============================================================================

from setuptools import find_packages, setup

package_name = 'collaborative_arm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luis-glez',
    maintainer_email='luis_glez2626@outlook.com',
    description='Computer vision node for detecting apples using stereo cameras',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This registers the vision_detector node so it can be run via ros2 run
            'vision_detector = collaborative_arm_vision.vision_detector:main',
        ],
    },
)