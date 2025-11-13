from setuptools import find_packages, setup

package_name = 'omega_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abel Mendoza',
    maintainer_email='abelxmendoza@gmail.com',
    description='The omega_robot package for ROS 2 Humble',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_publisher = omega_robot.telemetry_publisher:main',
            'telemetry_listener = omega_robot.telemetry_listener:main',
            'sensor_data_publisher = omega_robot.sensor_data_publisher:main',
            'robot_controller = omega_robot.robot_controller:main',
            'enhanced_telemetry = omega_robot.enhanced_telemetry:main',
        ],
    },
)

