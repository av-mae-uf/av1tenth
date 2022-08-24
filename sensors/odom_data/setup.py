from setuptools import setup

package_name = 'odom_data'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Penumarti',
    maintainer_email='apenumarti@ufl.edu',
    description='Package for publishing IMU Data, which includes, angle off true north, angular acceleration and velocity, etc.',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_pub = odom_data.odom_pub:main'
        ],
    },
)
