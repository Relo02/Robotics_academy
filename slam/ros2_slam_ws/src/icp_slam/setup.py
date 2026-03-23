from setuptools import setup
import os
from glob import glob

package_name = 'icp_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ICP scan-matching and mapping node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_node = icp_slam.icp_node:main',
        ],
    },
)
