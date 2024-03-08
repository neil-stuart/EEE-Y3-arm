from setuptools import setup, find_packages

setup(
    name='g11_moveo',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'pyserial',
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='A Python package to control the BCN3D Moveo robotic arm.',
    keywords='BCN3D Moveo, robotic arm, control'
)
