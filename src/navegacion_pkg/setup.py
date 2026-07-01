from setuptools import find_packages, setup

package_name = 'navegacion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        ['launch/simulation.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/tp_final.rviz']),
        ('share/' + package_name, ['map.yaml', 'map.pgm']), # ponerlo o no ponerlo ? ***
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romanella',
    maintainer_email='romacolombini@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_node = navegacion_pkg.robot:main',
            'localization_node = navegacion_pkg.localization_node:main',
            'likelihood_field_node = navegacion_pkg.likelihood_field:main',
            'map_publisher = navegacion_pkg.map_publisher:main',
        ],
    },
)
