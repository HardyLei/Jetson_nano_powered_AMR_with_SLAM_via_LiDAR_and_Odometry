"""Install AMR bring-up launch files and configuration."""

from glob import glob

from setuptools import setup

package = 'amr_bringup'

setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package}']),
        (f'share/{package}', ['package.xml']),
        (f'share/{package}/launch', glob('launch/*.launch.py')),
        (f'share/{package}/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    maintainer='uhang',
    maintainer_email='luh0519@connect.hku.hk',
    description='Bring-up and mapping for the mecanum AMR',
    license='Apache-2.0',
)
