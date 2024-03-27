from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neil Stuart',
    maintainer_email='n.stuart3@universityofgalway.ie',
    description='Package for image processing and motor control of robotic arm for Group 11 third year project 2023/24.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = arm_pkg.image_processor:main',
            'controls_generator = arm_pkg.controls_generator:main'
        ],
    },
)
