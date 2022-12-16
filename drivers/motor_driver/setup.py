from setuptools import setup

package_name = "motor_driver"

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
    maintainer="aditya",
    maintainer_email="apenumarti@ufl.edu",
    description="This package runs the actuators on the vehicle. It uses the Pololu Micro Maestro servo controller to accomplish this.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_controller = motor_driver.motor_controller:main",
        ],
    },
)
