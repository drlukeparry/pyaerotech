PyAerotech: An Opensource Python Library or interfacing directly with the Aerotech A3200 Controller
========================================================================


pyAerotech is an additional Opensource Python library for interfacing with the Aerotech A3200 Controller use to drive linear stages and gavlo mirrors.
The library has been developed primarily  to support the development of research lab systems for use in 3D printing, which require additional control.
Additionaly it aims to simplify integration amongst other software routines and micro-controllers.

The library provide a vanilla c++ interface using theb*std::library*, alongside the Eigen library and the Python bindings are generated
via `pybind <https://pybind11.readthedocs.io/en/stable/>`_. The library is not intended to provide full compatability with the Aerotech API nor full
capability offered but aims to offer convenient access to  the majority of its features.

Note this work has been developed independently from Aerotech, nor is it affiliated. Please

Installation
#################


No strict dependencies are required for compiling pyaerotech, except a distribution of the original A3200 API Library - available with a valid license for the A3200 Controller.
The Python bindings are built on top of the C library for interfacing with the controller and aim to provide a convenient access to functions.

Python bindings are generated via
`pybind <https://pybind11.readthedocs.io/en/stable/>`_, which is automatically pulled in by as sub-module by calling
`git clone with `--recursive`.


.. code:: bash

    git clone --recursive https://github.com/pyaerotech
    cmake .


Compiler Requirements
**********************
Pyaerotech was designed to not include many dependencies to improve the compatibility to integrate into existing python programs.
The underlying library provided by Aerotech for the A3200 controller only supports Windows, therefore compilation can only b

**On Windows**

* Visual Studio 2015 (Cxx14 version required for all Python versions)
* CMake >= 3.1
* A3200 Controller Software

During the build process both dynamic and static libraries are generated.


Installation: Python Bindings - Compiling from Source
********************************************************

The python module can be generated using python by simply cloning this repository and then running pip install
in your python environment. The Aerotech A3200 C API library requires copying into the `external\a3200` folder to corretly
compile. Typically the folder is located in `\Aerotech\A3200\CLibrary`

Note the `--recursive` option which is needed for the `pybind11`, `eigen`, and `filesystem`
submodule:

.. code:: bash

    git clone --recursive https://github.com/pyaerotech
    pip install ./pyaerotech


With the `setup.py` file included in this example, the `pip install` command will invoke CMake to build the pybind11
module as specified in `CMakeLists.txt` and generate a package. A specific version of python is not required provided
it is compatible with pybind. During the process The CMake Option flag `BUILD_PYTHON` will be automatically toggled on
during the build phase.

Unfortunatly a Python precompiled python package cannot be distributed as the Aerotech API library cannot be packaged
and distributed.

Usage: Python
#################

Please refer to the provided examples and additionally the required inputs within the Aerotech A3200 Programming Manual.
