import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tail_control'

setup(
    name='tail_control',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='rjmpoon@gmail.com',
    description='Control everything on the tail',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'killswitch_relay = tail_control.killswitch_relay:main',
        ],
    },
)
