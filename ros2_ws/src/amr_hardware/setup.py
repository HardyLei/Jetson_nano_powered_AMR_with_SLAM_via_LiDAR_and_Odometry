from setuptools import find_packages, setup

package_name = 'amr_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luh0519',
    maintainer_email='luh0519@connect.hku.hk',
    description='this package is for the serial communication between\
        Jetson nano and Arduino Mega of the AMR robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_interface_node = amr_hardware.serial_interface_node:main',
        ],
    },
)
