from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'gesture_pointer'
submodules = 'gesture_pointer/submodules'
utils = 'gesture_pointer/utils'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, submodules, utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))      
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nmksas',
    maintainer_email='noora.sassali@gmail.com',
    description='Gesture pointer',
    license='Apache License v2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'gesture_pointer = gesture_pointer.gesture_pointer_node:main', 
        ]
    },
)
