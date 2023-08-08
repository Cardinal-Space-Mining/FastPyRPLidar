# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import glob
import platform
from os.path import join as path_join
import os
from distutils.ccompiler import new_compiler

#TODO Consider removing the library source path from includes and moving to the sl_* convention
Slamtek_SDK_include_path =  [path_join(".","SlamtekSDK","sdk","include")]

def get_static_lib_ext():
    if platform.system() == "Linux":
        return ".a"
    elif platform.system() == "Windows":
        return ".lib"
    elif platform.system() == "Darwin":
        return ".a"
    else:
        raise OSError("Invalid OS Detected")
    
def get_static_lib_prefix():
    if platform.system() in ("Linux", "Darwin"):
        return "lib"
    elif platform.system() == "Windows":
        return ""
    else:
        raise OSError("Invalid OS Detected")

def make_RPLidar_SDK():
    # The slamtek include headers interfere with either the pybind11 headers or the Python.h file
    # Best solution I got is to compile the slamtek file staticaly and compile that in
    shared_lib_name = "SlamtekSDK"
    full_lib_name = os.path.abspath(path_join(".", get_static_lib_prefix() + shared_lib_name + get_static_lib_ext()))
    if os.path.exists(full_lib_name):
        return full_lib_name # Already built, don't need a repeat


    slamtek_include_dirs = [
        *Slamtek_SDK_include_path,   
        path_join(".","SlamtekSDK","sdk","src")
    ]

    slamtek_src_files = [
        *glob.glob(path_join('.', 'SlamtekSDK', 'sdk', 'src', '*.cpp')),
        path_join('.', 'SlamtekSDK', 'sdk', 'src', 'hal', 'thread.cpp')
    ]

    if platform.system() == "Linux":
        slamtek_include_dirs.append(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'linux'))
        slamtek_src_files.extend(glob.glob(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'linux', "*.cpp")))
    elif platform.system() == "Windows":
        slamtek_include_dirs.append(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'win32'))
        slamtek_src_files.extend(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'win32', "*.cpp"))
    elif platform.system() == "Darwin":
        slamtek_include_dirs.append(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'macOS'))
        slamtek_src_files.extend(glob.glob(path_join('.', 'SlamtekSDK', 'sdk', 'src', 'arch', 'macOS', "*.cpp")))
    else:
        raise OSError("Invalid OS Detected")
    
    comp = new_compiler()

    for dir in slamtek_include_dirs:
        comp.add_include_dir(dir)

    slamtek_src_files.sort()

    if comp.compiler_type == "msvc":
        # Position independent code is on by default
        objects = comp.compile(slamtek_src_files)
    else:
        # -fPIC is required for the compilation of the shared library
        objects = comp.compile(slamtek_src_files, extra_postargs=["-fPIC"])


    comp.create_static_lib(objects, shared_lib_name, output_dir=".")
    
    return full_lib_name

__version__ = "0.0.1"

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)


ext_modules = [
    Pybind11Extension(
        "FastRPLidar",
        sorted([*glob.glob("./src/*.cpp") ]),
        include_dirs = ["./src", *Slamtek_SDK_include_path],
        define_macros = [('VERSION_INFO', __version__)],
        extra_objects = [make_RPLidar_SDK()]
        ),
]

setup(
    name="FastRPLidar",
    version=__version__,
    author="William Mosier",
    author_email="willmoiser@gmail.com",
    url="https://github.com/wimos-ai/FastRPLidar",
    description="A light weight and fast set of python bindings to the SlamTek RPLidar SDK",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    test_suite="tests",
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)