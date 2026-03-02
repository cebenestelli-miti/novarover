from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mower_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corye',
    maintainer_email='corye@todo.todo',
    description='Localization: fuse odom/raw (and optionally GPS/IMU), publish filtered odom and TF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = mower_localization.localization_node:main',
        ],
    },
)
