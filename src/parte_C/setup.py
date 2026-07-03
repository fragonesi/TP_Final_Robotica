from setuptools import find_packages, setup

package_name = 'tpf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        ['launch/simulation.launch.py',
         'launch/robot_real.launch.py']),
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
            'robot_node = tpf.robot:main',
            'robot_node_c = tpf.robot_parte_c:main',
            'localization_node = tpf.localization_node:main',
            'localization_node_tb4 = tpf.localization_node_tb4:main',
            'likelihood_field_node = tpf.likelihood_field:main',
            'map_publisher = tpf.map_publisher:main',
        ],
    },
)
