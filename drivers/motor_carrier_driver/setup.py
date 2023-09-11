import os
from glob import glob
from setuptools import setup

package_name = "motor_carrier_driver"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Patrick Neal, Aditya Penumarti",
    maintainer_email="apenumarti@ufl.edu",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = motor_carrier_driver.motor_carrier_serial_driver:main"],
    },
)
