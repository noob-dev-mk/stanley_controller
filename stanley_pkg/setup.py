from setuptools import setup
import os
from glob import glob

package_name = 'stanley_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MK',
    maintainer_email='root@todo.todo',
    description='Here we will implement stanley control algo for f1tenth',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_check = stanley_pkg.odom_check:main',
            'vel_publisher = stanley_pkg.vel_publisher:main',
            'stanley_v1 = stanley_pkg.stanley_v1:main',
            'stanley_final = stanley_pkg.stanley_final:main'
        ],
    },
)
