cmake_minimum_required(VERSION 3.14)
project(remani_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.11.2 REQUIRED)

# Core library
add_library(remani_core SHARED
    src/kinematic_model.cpp
    src/mdh_model.cpp
    src/poe_model.cpp
    src/urdf_model.cpp
    src/delta_model.cpp
    src/model_manager.cpp
    src/instance_manager.cpp
    src/kinematic_solver.cpp
    src/dynamics_solver.cpp
    src/safety_checker.cpp
    src/math_utils.cpp
    src/config_store.cpp
)

target_include_directories(remani_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(remani_core
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

# Install rules
install(TARGETS remani_core
    EXPORT remani_coreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/remani DESTINATION include)
