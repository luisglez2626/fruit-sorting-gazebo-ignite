# ==============================================================================
# File: setup.py
# Purpose: Python package setup script. It tells ROS 2 how to install the 
#          Python modules and defines the executable entry points for the GUI.
# ==============================================================================

from setuptools import find_packages, setup

package_name = 'collaborative_arm_application'

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
    description='GUI application for manual Cartesian control and vision interaction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registers the cartesian_jogger node so it can be run via ros2 run
            'cartesian_jogger = collaborative_arm_application.cartesian_jogger:main',
        ],
    },
)