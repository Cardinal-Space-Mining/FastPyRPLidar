from setuptools import setup
import os


def find_stubs(package):
    stubs = []
    for root, dirs, files in os.walk(package):
        for file in files:
            path = os.path.join(root, file).replace(package + os.sep, '', 1)
            stubs.append(path)
    return dict(package=stubs)


setup(
    name='FastRPLidar-stubs',
    maintainer="FastRPLidar Developers",
    maintainer_email="example@python.org",
    description="PEP 561 type stubs for FastRPLidar",
    version='1.0',
    packages=['FastRPLidar-stubs'],
    # PEP 561 requires these
    install_requires=['FastRPLidar'],
    package_data=find_stubs('FastRPLidar-stubs'),
)