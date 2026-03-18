from setuptools import setup
from glob import glob

package_name = 'coverage_replanner'

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
    description='Local replanner service for mode D (partial replanning).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_replanner = coverage_replanner.local_replanner_node:main',
            'region_state = coverage_replanner.region_state_node:main',
            'd1_advisor = coverage_replanner.d1_advisor_node:main',
            'd2_advisor = coverage_replanner.d2_advisor_node:main',
        ],
    },
)
