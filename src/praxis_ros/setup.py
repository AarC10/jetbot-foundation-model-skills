from setuptools import find_packages, setup

package_name = "praxis_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aaron Chan",
    maintainer_email="amc9897@rit.edu",
    description="ROS 2 bridge for Praxi",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "skill_node = praxis_ros.skill_node:main",
        ],
    },
)
