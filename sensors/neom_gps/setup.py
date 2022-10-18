from setuptools import setup

package_name = "neom_gps"

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
    maintainer="Carl Crane, Patrick Neal, Aditya Penumarti",
    maintainer_email="carl.crane@gmail.com, neap@ufl.edu, apenumarti@ufl.edu",
    description="Package for publishing GPS data from a NEO-6M GPS unit. Can be used for navigation, designed for autonomous route point navigation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = neom_gps.neo6m_driver_node:main"],
    },
)
