from setuptools import find_packages, setup

package_name = 'xaxa0404'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wizymory',
    maintainer_email='wizymory@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "write_single_reg = xaxa0404.ebyte_modbus:main",
            "coba1 = xaxa0404.coba_0:main",
            "publisher_xaxa0404 = xaxa0404.modbus_ui_node:main",
            "subscriber_xaxa0404 = xaxa0404.modbus_executor_node:main",
            "pymodbus_xaxa0404 = xaxa0404.pymodbus_executor:main"
        ],
    },
)
