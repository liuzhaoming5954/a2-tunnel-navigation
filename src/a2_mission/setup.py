from setuptools import find_packages, setup


setup(
    name="a2_mission",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/a2_mission"]),
        ("share/a2_mission", ["package.xml"]),
    ],
    install_requires=["setuptools", "a2_storage"],
    zip_safe=True,
    maintainer="Project Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Offline mission admission and recovery rules for A2",
    license="LicenseRef-Proprietary",
    python_requires=">=3.10",
    entry_points={"console_scripts": [
        "a2-mission-demo = a2_mission.demo:main",
        "mission_node = a2_mission.ros_node:main",
    ]},
)
