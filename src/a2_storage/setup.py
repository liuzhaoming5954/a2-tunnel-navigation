from setuptools import find_packages, setup


setup(
    name="a2_storage",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/a2_storage"]),
        ("share/a2_storage", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Local transactional journal for the A2 offline mission core",
    license="LicenseRef-Proprietary",
    python_requires=">=3.10",
)
