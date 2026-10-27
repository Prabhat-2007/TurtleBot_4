from setuptools import find_packages, setup

package_name = 'tb4_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb4 maintainer',
    maintainer_email='gorantlaprabhat18@gmail.com',
    description='Driving helpers for the TurtleBot 4',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = tb4_drive.keyboard_teleop:main',
            'velocity_gate = tb4_drive.velocity_gate:main',
            'sensor_demo = tb4_drive.sensor_demo:main',
        ],
    },
)
