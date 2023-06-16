from setuptools import setup

package_name = 'px4_ros_mas_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vasco',
    maintainer_email='vasco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_liftoff = px4_ros_mas_com.drone_liftoff:main',
            'single_drone_liftoff = px4_ros_mas_com.single_drone_liftoff:main',
            'drone_offboard_control = px4_ros_mas_com.drone_offboard_control:main',
            'listener_drone_pos = px4_ros_mas_com.listener_drone_pos:main'
        ],
    },
)
