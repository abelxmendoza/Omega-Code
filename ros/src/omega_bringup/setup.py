from setuptools import setup
import os
from glob import glob

package_name = 'omega_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        # Install all config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.xml')) +
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abel Mendoza',
    maintainer_email='abelxmendoza@gmail.com',
    description='Omega-1 bringup package: launch files and system config',
    license='BSD',
    entry_points={'console_scripts': []},
)
