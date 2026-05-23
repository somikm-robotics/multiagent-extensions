from setuptools import setup

package_name = 'turtlebot'

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
    description='Turtlebot agent node package for multi-agent extension project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wait_for_tf_node = turtlebot.nodes.wait_for_tf_node:main',
            'scan_fresh_relay_node = turtlebot.nodes.scan_fresh_relay_node:main',
            'orbit_twist_node = turtlebot.nodes.orbit_twist_node:main',
            'orbit_twist_corrected_node = turtlebot.nodes.orbit_twist_corrected_node:main',
            'orbit_manager_node = turtlebot.nodes.orbit_manager_node:main'
        ],
    },
)
