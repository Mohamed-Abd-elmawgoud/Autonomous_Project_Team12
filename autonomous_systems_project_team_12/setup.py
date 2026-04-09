from setuptools import find_packages, setup

package_name = 'autonomous_systems_project_team_12'

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
            "Validation_Printing_Node_Team_12 = autonomous_systems_project_team_12.Validation_Printing_Node_Team_12:main",
            "olr_node = autonomous_systems_project_team_12.olr_node:main",
            "keyboard_reader = autonomous_systems_project_team_12.keyboard_reader:main",
            "vehicle_state = autonomous_systems_project_team_12.vehicle_state:main",
        ],
    },
)
