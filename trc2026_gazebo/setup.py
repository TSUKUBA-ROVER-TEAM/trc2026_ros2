from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'trc2026_gazebo'

field_red_files = [f for f in glob('models/field_red/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/field_red'), field_red_files),
        (os.path.join('share', package_name, 'models/field_red/meshes'),
         glob('models/field_red/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kengo',
    maintainer_email='kengo171227@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
