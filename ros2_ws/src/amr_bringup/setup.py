from pathlib import Path

from setuptools import setup


package_name = 'amr_bringup'
package_dir = Path(__file__).resolve().parent


def package_files(*patterns):
    # Colcon runs this symlinked setup.py in a build directory that can retain
    # links to files removed from the source checkout.
    return sorted(
        str(path.relative_to(package_dir))
        for pattern in patterns
        for path in package_dir.glob(pattern)
        if path.is_file()
    )


setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', package_files('launch/*.launch.py')),
        ('share/' + package_name + '/config', package_files('config/*.yaml', 'config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uhang',
    maintainer_email='luh0519@connect.hku.hk',
    description='Launch files for bringing up the AMR hardware and sensors.',
    license='Apache-2.0',
)
