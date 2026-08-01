from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'multi_agent_simulation'

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
    maintainer='root',
    maintainer_email='root@googlemail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_covariance_injector_node = multi_agent_simulation.nodes.gps_covariance_injector_node:main'
            'imu_covariance_injector_node = multi_agent_simulation.nodes.imu_covariance_injector_node:main'
        ],
    },
)
