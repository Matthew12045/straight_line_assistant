import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'straight_line_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description=(
        'Teleop assistant that holds heading using odometry + PID during '
        'straight-line driving.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_line_assistant_node = '
            'straight_line_assistant.straight_line_assistant_node:main',
            'pid_dashboard = '
            'straight_line_assistant.pid_dashboard:main',
            'ekf_node = '
            'straight_line_assistant.ekf_node:main',
        ],
    },
)
