from setuptools import find_packages, setup

package_name = 'human_robot_collab'

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
    maintainer='hazem',
    maintainer_email='a214.shams@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'hand_to_collision = human_robot_collab.hand_to_collision:main',
             'validate_corners = human_robot_collab.validate_corners:main',
             'waypoint_manager = human_robot_collab.waypoint_manager:main',
        ],
    },
)
