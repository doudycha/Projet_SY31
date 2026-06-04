from setuptools import find_packages, setup
from glob import glob

package_name = 'tp5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doudycha',
    maintainer_email='doudycha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'project_node = tp5.project:main',
            'detect_node = tp5.detect:main',
        ],
    },
)
