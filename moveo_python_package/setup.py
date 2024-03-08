from setuptools import setup, find_packages

setup(
    name='g11_moveo',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'pyserial',
    ],
    author='Neil Stuart',
    author_email='n.stuart3@universityofgalway.ie',
    description='A Python package to control the BCN3D Moveo robotic arm.',
    keywords='BCN3D Moveo, robotic arm, control'
)
