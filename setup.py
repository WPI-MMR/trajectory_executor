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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generator = trajectory_executor.generator:main',
            'tracker = trajectory_executor.tracker:main',
            'com_serial = trajectory_executor.com_serial:main',
            'com_socket = trajectory_executor.com_socket:main'
        ],
    },
)
