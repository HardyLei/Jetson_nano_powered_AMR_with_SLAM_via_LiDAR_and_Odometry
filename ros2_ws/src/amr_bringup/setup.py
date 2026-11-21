from glob import glob

from setuptools import setup


package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uhang',
    maintainer_email='luh0519@connect.hku.hk',
    description='Launch files for bringing up the AMR hardware and sensors.',
    license='Apache-2.0',
)
