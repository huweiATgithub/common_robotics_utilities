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
    long_description=long_description,
    long_description_content_type="text/markdown",
)
