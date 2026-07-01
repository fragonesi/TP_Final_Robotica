from setuptools import find_packages, setup

package_name = 'aruco_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['aruco_pkg/noise_model.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@example.com',
    description='Detección y estimación de pose de marcadores ArUco - TP Final Parte A (Opción 3)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'aruco_detector_node = aruco_pkg.aruco_detector_node:main',
        'odom_delta_node = aruco_pkg.odom_delta_node:main',
    ],
},
)
