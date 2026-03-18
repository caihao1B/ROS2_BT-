from setuptools import setup
from glob import glob

package_name = 'coverage_metrics'

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
    description='Coverage task metrics for Milestone1: coverage ratio, overlap ratio, stuck events, segment failure rate.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_metrics = coverage_metrics.metrics_node:main',
        ],
    },
)
