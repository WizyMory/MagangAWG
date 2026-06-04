import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'amr_nav_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('amr_nav_interface/static/*')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn'],
    zip_safe=True,
    maintainer='wizymory',
    maintainer_email='wizymory@todo.todo',
    description='Web interface bridge for AMR point-to-point Nav2 navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_web_server = amr_nav_interface.nav_web_server:main',
        ],
    },
)
