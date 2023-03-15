import os
from setuptools import setup
from glob import glob

package_name = 'vehicle_launch'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Penumarti',
    maintainer_email='apenumarti@ufl.edu',
    description='Master Car Launch Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
