from setuptools import setup
from glob import glob

package_name = 'coverage_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Perception (LaserScan clustering/tracking) for dynamic obstacles.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_tracker = coverage_perception.scan_tracker_node:main',
        ],
    },
)
