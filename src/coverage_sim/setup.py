from setuptools import setup
from glob import glob

package_name = 'coverage_sim'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robust Gazebo Classic dynamic obstacle generator (agriculture-oriented scenarios).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dynamic_obstacles = coverage_sim.dynamic_obstacles_node:main',
        ],
    },
)
