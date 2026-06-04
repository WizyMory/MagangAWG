from setuptools import setup, find_packages
import os

package_name = 'amr_relay'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'relay_node = amr_relay.relay_node:main',
        ],
    },
)
