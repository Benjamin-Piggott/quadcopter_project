from setuptools import setup

package_name = 'quadcopter_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='your_email@example.com',
    description='Quadcopter navigation project using ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_config_service = quadcopter_project.grid_config_service_node:main',
            'pathfinding_service = quadcopter_project.pathfinding_service_node:main',
            'motor_control = quadcopter_project.motor_control_node:main',
        ],
    },
)
