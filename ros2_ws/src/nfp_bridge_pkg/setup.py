from setuptools import find_packages, setup

package_name = 'nfp_bridge_pkg'

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
    maintainer='tanishk',
    maintainer_email='ctanishk8@gmail.com',
    description='NFP to Nav2 Bridge Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_cop = nfp_bridge_pkg.traffic_cop:main',
            'fleet_node = nfp_bridge_pkg.fleet_node:main',
        ],
    },
)
