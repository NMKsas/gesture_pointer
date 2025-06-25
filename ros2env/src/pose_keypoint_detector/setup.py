from setuptools import find_packages, setup

package_name = 'pose_keypoint_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nmksas',
    maintainer_email='noora.sassali@gmail.com',
    description='Pose keypoint detector package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_pose_detector = pose_keypoint_detector.yolo11_keypoint_detector:main',
        ],
    },
)
