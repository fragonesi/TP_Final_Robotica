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
        ['launch/simulation.launch.py', 'launch/robot_real.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/tp_final.rviz']),
        # map.yaml/.pgm: mapa real (Parte A), usado por robot_real.launch.py.
        # map_sim.yaml/.pgm: mapa a escala de Gazebo (casa.world), usado por
        # simulation.launch.py -- son mundos de tamaño muy distinto (~13x11 m
        # vs ~33x33 m), así que no pueden compartir un solo mapa.
        ('share/' + package_name, ['map.yaml', 'map.pgm', 'map_sim.yaml', 'map_sim.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romanella',
    maintainer_email='romacolombini@gmail.com',
    description='Navegación autónoma (localización por filtro de partículas, Theta*, Pure Pursuit, máquina de estados) — TP Final Parte B',
    license='Apache-2.0',
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
