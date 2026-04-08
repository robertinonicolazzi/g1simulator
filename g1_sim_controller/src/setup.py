from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g1_sim_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rober',
    maintainer_email='rober@todo.todo',
    description='Simple printer node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_telemetry = g1_sim_controller.basic_telemetry:main',
            # 'cmd_vel_bridge = g1_sim_controller.cmd_vel_bridge:main',  # disabled – replaced by dds_ros2_bridge
            'dds_ros2_bridge = g1_sim_controller.dds_ros2_bridge:main',
            'camera_bridge = g1_sim_controller.camera_bridge:main'
        ]
    },
)
