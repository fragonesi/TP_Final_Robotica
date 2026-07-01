import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'TP_Final_Robotica'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romanella',
    maintainer_email='romacolombini@gmail.com',
    description='GraphSLAM con landmarks ArUco + odometría — TP Final Parte A (Opción 3)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'graph_slam_node = TP_Final_Robotica.graph_slam_node:main',
            'scan_logger_node = TP_Final_Robotica.scan_logger_node:main',
        ],
    },
)
