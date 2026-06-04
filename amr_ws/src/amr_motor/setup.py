from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_motor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # config files
       # ('share/' + package_name + '/config',
        #    ['config/motor.yaml']),
    ],
    install_requires=[
        'setuptools',
        'pymodbus',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='rodik.w.i@ftmm.unair.ac.id',
    description='Industrial AMR motor driver with Modbus RTU, autoreconnect, diagnostics',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # format: name = module:function
            'amr_motor_node = amr_motor.amr_motor_node:main',
            'move_motor = amr_motor.move_motor:main',
        ],
    },
)
