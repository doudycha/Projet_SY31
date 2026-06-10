from setuptools import find_packages, setup
from glob import glob

package_name = 'projet2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doudycha',
    maintainer_email='doudycha@todo.todo',
    description='SY31 Projet 2 - Cartographie guidee dans un labyrinthe',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'odometry          = projet2.odometry:main',
            'mapper            = projet2.mapper:main',
            'arrow_detector    = projet2.arrow_detector:main',
            'direction_display = projet2.direction_display:main',
        ],
    },
)
