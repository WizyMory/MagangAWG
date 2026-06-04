from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_user_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static',
            glob('amr_user_interface/static/*')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='rodik.w.i@ftmm.unair.ac.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amr_web_server = amr_user_interface.amr_web_server:main',
        ],
    },
)
