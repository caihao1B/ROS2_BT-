from setuptools import setup
import os
from glob import glob

package_name = 'coverage_manager'

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
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Segment-level coverage executor for Nav2 FollowPath with recovery/retry/skip and event/state logging.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_executor = coverage_manager.executor_node:main',
            'coverage_executor_guarded = coverage_manager.executor_guarded_node:main',
            'coverage_executor_nobt_rejoin = coverage_manager.executor_nobt_rejoin_node:main',
        ],
    },
)
