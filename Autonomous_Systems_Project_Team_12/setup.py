from setuptools import find_packages, setup

package_name = 'Autonomous_Systems_Project_Team_12'

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
    maintainer='dell',
    maintainer_email='mohmedsayed0990@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "Validation_Printing_Node_Team_12 = Autonomous_Systems_Project_Team_12.Validation_Printing_Node_Team_12:main",
            "Testing_Node = Autonomous_Systems_Project_Team_12.Testing_Node:main",
            "teleop_vehicle_controller = Autonomous_Systems_Project_Team_12.teleop_vehicle_controller:main",
            "teleop_press_hold_node = Autonomous_Systems_Project_Team_12.teleop_press_hold_node:main",
        ],
    },
)
