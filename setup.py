from setuptools import setup

package_name = 'trajectory_executor'

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
    maintainer='Andrew Euredjian, Ankur Gupta, Revant Mahajan',
    maintainer_email='ageuredjian@wpi.edu, agupta4@wpi.edu, rmahajan@wpi.edu',
    description='ROS package that handles all aspects required in executing a trajectory',
    license='MIT',
    tests_require=['pytest', 'parameterized'],
    entry_points={
        'console_scripts': [
            'generator = trajectory_executor.generator:main',
            'tracker = trajectory_executor.tracker:main',
            'com_serial = trajectory_executor.com_serial:main',
            'com_socket = trajectory_executor.com_socket:main',
            'serial_test = trajectory_executor.serial_test:main',
            'odrive_test = trajectory_executor.odrive_test:main',
            'traj_test = trajectory_executor.traj_test:main',
            'trot = trajectory_executor.trot:trot',
            'calibrate = trajectory_executor.trot:calibrate',
        ],
    },
)
