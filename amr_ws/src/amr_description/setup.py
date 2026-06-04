from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='rodik.w.i@ftmm.unair.ac.id',
    description='AMR Robot Description (URDF, Xacro, TF, Parameters)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
