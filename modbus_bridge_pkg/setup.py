from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'modbus_bridge_pkg'
yaml_config = (os.path.join('share', package_name, 'config'), ['config/me31_params.yaml'])

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]*'))),
        yaml_config
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zalfa',
    maintainer_email='zalfa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_node = modbus_bridge_pkg.modbus_node:main',
            'controller = modbus_bridge_pkg.controller:main',
            'gui_node = modbus_bridge_pkg.gui_node:main'
        ],
    },
)
