from setuptools import find_packages, setup

package_name = 'cono_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zoe',
    maintainer_email='zvelazquezzorzi@udesa.edu.ar',
    description='Deteccion de conos rojos por color + fusion con LIDAR (Parte C)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_cono = cono_detector_pkg.detector_cono_node:main',
        ],
    },
)
