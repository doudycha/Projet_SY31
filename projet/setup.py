from setuptools import find_packages, setup
from glob import glob

package_name = 'projet'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Projet SY31 - Labyrinthe',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detector_node         = projet.detector:main',
            'transformer_node      = projet.transformer:main',
            'intensity_filter_node = projet.intensity_filter:main',
            'odompose_node         = projet.odompose:main',
            'pipeline_node         = projet.pipeline:main',
        ],
    },
)
