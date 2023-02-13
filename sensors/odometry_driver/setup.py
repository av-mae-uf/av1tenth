from setuptools import setup

package_name = "odometry_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aditya Penumarti",
    maintainer_email="apenumarti@ufl.edu",
    description="Package for publishing IMU Data, which includes, angle off true north, angular acceleration and velocity, etc.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = odometry_driver.odometry_driver:main"],
    },
)
