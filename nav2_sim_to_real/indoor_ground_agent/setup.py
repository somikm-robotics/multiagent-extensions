from setuptools import setup

package_name = 'indoor_ground_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Indoor Ground Agent agent node package for multi-agent extension project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_relay_node = indoor_ground_agent.nodes.twist_relay_node:main',
            'orbit_generator = indoor_ground_agent.nodes.orbit_generator:main',
            'orbit_manager = indoor_ground_agent.nodes.orbit_manager:main',
        ],
    },
)
