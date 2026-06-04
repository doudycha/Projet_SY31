from setuptools import find_packages, setup

package_name = 'tp4'

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
                'clusterer = tp4.clusterer:main',
                'intensity_filter = tp4.intensity_filter:main',
                'shaper_bbox = tp4.shaper_bbox:main',
                'shaper_cylinder = tp4.shaper_cylinder:main',
                'shaper_polyline = tp4.shaper_polyline:main',
                'transformer = tp4.transformer:main',
                'utils = tp4.utils:main',
        ],
    },
)
