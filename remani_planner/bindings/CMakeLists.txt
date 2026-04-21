cmake_minimum_required(VERSION 3.14)
project(remani_bindings VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# Find core library
find_package(remani_core REQUIRED)

# Python module
pybind11_add_module(remani_core_py
    src/py_kinematic_params.cpp
    src/py_model.cpp
    src/py_managers.cpp
    src/py_solvers.cpp
)

target_link_libraries(remani_core_py PRIVATE remani_core)

target_include_directories(remani_core_py PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set output directory to Python package location
set_target_properties(remani_core_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/remani
)

# Install Python module
install(TARGETS remani_core_py
    LIBRARY DESTINATION ${Python3_SITEARCH}/remani
)
