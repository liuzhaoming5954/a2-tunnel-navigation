from glob import glob

from setuptools import find_packages, setup


setup(
    name="a2_bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/a2_bringup"]),
        ("share/a2_bringup", ["package.xml"]),
        ("share/a2_bringup/launch", glob("launch/*.launch.py")),
        ("share/a2_bringup/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Hardware-free mock platform and mission launch composition",
    license="LicenseRef-Proprietary",
    python_requires=">=3.10",
)
