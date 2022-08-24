from setuptools import setup

package_name = 'gps_pub'

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
    maintainer='Carl Crane',
    maintainer_email='carl.crane@gmail.com',
    description='Package for publishing GPS data from a NEO-6M GPS unit. Can be used for navigation, designed for autonomous route point navigation',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_pub = gps_pub.gps_pub_function:main'
        ],
    },
)
