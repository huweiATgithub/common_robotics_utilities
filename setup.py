from setuptools import find_packages
from skbuild import setup
from pathlib import Path

this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    use_calver="%Y.%-m.%-d",
    package_dir={"": "bindings"},
    packages=find_packages(where="bindings"),
    cmake_install_dir="bindings",
    # specify dependencies in pyproject.toml is not supported yet.
    # [project] section requires name and version,
    # but specifying version using calver is not supported in pyproject.toml.
    install_requires=["numpy"],
    long_description=long_description,
    long_description_content_type="text/markdown",
)
