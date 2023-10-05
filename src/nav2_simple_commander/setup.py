from setuptools import find_packages, setup

package_name = 'nav2_simple_commander'

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
    maintainer='aliihsan',
    maintainer_email='aliihsangungoren1150@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_follow_path = nav2_simple_commander.example_follow_path:main',
            'example_waypoint_follower = nav2_simple_commander.example_waypoint_follower:main',
        ],
    },
)