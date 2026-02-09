from setuptools import find_packages, setup

package_name = 'patrol_mission'

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
    maintainer='hidayat',
    maintainer_email='hidayat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoints_follower = patrol_mission.waypoints_follower:main',
            'pose_navigator = patrol_mission.pose_navigator:main',
            'poses_navigator = patrol_mission.poses_navigator:main',
        ],
    },
)
