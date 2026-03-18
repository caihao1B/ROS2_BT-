from setuptools import setup
from glob import glob

package_name = 'farm_gate_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/bt', glob('bt/*.xml')),
        ('share/' + package_name + '/plans', glob('plans/*.json')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caihao1B',
    maintainer_email='',
    description='Farm-gate bringup (Gazebo + Nav2 + coverage executor) for ROS2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_initial_pose = farm_gate_bringup.initial_pose_publisher:main',
        ],
    },
)
