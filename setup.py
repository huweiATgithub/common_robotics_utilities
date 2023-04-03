from setuptools import find_packages
from skbuild import setup
from setuptools_scm import get_version


setup(
    version=get_version(),  # configure in pyproject.toml will enable isolated build
    package_dir={"": "bindings"},
    packages=find_packages(where="bindings"),
    cmake_install_dir="bindings",
)
