from setuptools import setup
import os
from glob import glob

package_name = 'cat_paddock_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'truck_100t'),
            glob('models/truck_100t/*')),
        (os.path.join('share', package_name, 'models', 'truck_250t'),
            glob('models/truck_250t/*')),
        (os.path.join('share', package_name, 'models', 'truck_400t'),
            glob('models/truck_400t/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanishk',
    maintainer_email='ctanishk8@gmail.com',
    description='Cat Tech Challenge 2026 paddock simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'truck_orchestrator = cat_paddock_sim.truck_orchestrator:main',
            'lidar_perception = cat_paddock_sim.lidar_perception:main',
            'hybrid_astar_node = cat_paddock_sim.hybrid_astar_node:main',
        ],
    },
)
