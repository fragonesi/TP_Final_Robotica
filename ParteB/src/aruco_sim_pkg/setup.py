from setuptools import find_packages, setup
import glob

package_name = 'aruco_sim_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eliana',
    maintainer_email='Elianaostro@gmail.com',
    description='Sensor virtual de landmarks ArUco para Gazebo - TP Final Parte B (Sistema 3)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_aruco_sensor_node = aruco_sim_pkg.virtual_aruco_sensor_node:main',
            'sim_odom_delta_node = aruco_sim_pkg.sim_odom_delta_node:main',
            'sim_scan_logger_node = aruco_sim_pkg.sim_scan_logger_node:main',
        ],
    },
)
