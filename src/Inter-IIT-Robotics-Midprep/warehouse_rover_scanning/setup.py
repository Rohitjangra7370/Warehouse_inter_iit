from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_rover_scanning'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Scanning controller and mission executor for warehouse robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanning_controller = warehouse_rover_scanning.scanning_controller:main',
            'mission_executor = warehouse_rover_scanning.mission_executor:main',
        ],
    },
)
