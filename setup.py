"""
Omega-Code backend as an editable Python package.

Run once after cloning so ROS nodes can import the hardware HAL
without sys.path manipulation or PYTHONPATH injection:

    pip install -e /path/to/Omega-Code/

After this, `from servers.robot_controller_backend.movement.xxx import Yyy`
works in any Python environment that has this package installed, including
inside colcon-built ROS nodes.
"""
from setuptools import setup, find_packages

# Explicitly list packages to avoid pulling in .direnv / venv / libcamera site-packages
PACKAGES = [
    'servers',
    'servers.robot_controller_backend',
    'servers.robot_controller_backend.api',
    'servers.robot_controller_backend.autonomy',
    'servers.robot_controller_backend.controllers',
    'servers.robot_controller_backend.controllers.lighting',
    'servers.robot_controller_backend.movement',
    'servers.robot_controller_backend.movement.hardware',
    'servers.robot_controller_backend.network',
    'servers.robot_controller_backend.network.api',
    'servers.robot_controller_backend.network.cli',
    'servers.robot_controller_backend.network.diagnostics',
    'servers.robot_controller_backend.network.vpn',
    'servers.robot_controller_backend.network.wifi',
    'servers.robot_controller_backend.network.wizard',
    'servers.robot_controller_backend.omega_config',
    'servers.robot_controller_backend.omega_services',
    'servers.robot_controller_backend.utils',
    'servers.robot_controller_backend.video',
]

setup(
    name='omega-backend',
    version='0.1.0',
    description='Omega-1 robot hardware abstraction layer',
    packages=PACKAGES,
    python_requires='>=3.10',
    install_requires=[
        'websockets>=12.0',
        'fastapi',
        'uvicorn',
        'smbus2',
    ],
    extras_require={
        'pi': ['lgpio', 'picamera2'],
        'ros': ['opencv-python'],
    },
)
