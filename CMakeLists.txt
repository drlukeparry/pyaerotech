cmake_minimum_required(VERSION 3.0)

project(pyAeroTech VERSION 0.1.0)

# Set c++ to use cx11 as a requirement
set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR OFF)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cMake")
# Instruct CMake to run moc automatically when needed.

# Copy the include directory if specified
set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}/aerotech")

# Generate DLL Pragma for creating shared libraries in Windows
include (GenerateExportHeader)

add_definitions("-DPROJECT_VERSION=\"${PROJECT_VERSION}\"" )

option(BUILD_PYTHON "Builds a python extension" OFF)

if(WIN32)
    # Remove Security definitions for the library
    # Remove run time checks for windows
    IF(MSVC)

        set(COMMON_LANGUAGE_RUNTIME "")
        set(EIGEN3_INCLUDE_DIR External/eigen)

    endif(MSVC)

    # Copy the shared libraries and libs to the main directory
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

else(WIN32)

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

    # Find Required packages
    find_package(Eigen3 3.4 REQUIRED)

endif(WIN32)


# Use the replacement of Boost::filesystem from a git submodule provided by WJakob
# in order to reduce compile time dependencies
# https://github.com/wjakob/filesystem

set(CPP_FILESYSTEM
    External/filesystem
)

## A replacement library for std::Format
set(CPP_FMT
    External/fmt/include
)

add_definitions("-DFMT_HEADER_ONLY=1")

# Set the Aerotech Include Libraries
set(AEROTECH_A3200_LIB_DIR
    External/a3200/Lib64
)

set(AEROTECH_A3200_INCLUDE_DIR
    External/a3200/Include
)


if(BUILD_PYTHON)
    message(STATUS "Building PyAeroTech Python Module")
    add_subdirectory(External/pybind11)

    set(PYBIND11_CPP_STANDARD /std:c++14)


endif(BUILD_PYTHON)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CPP_FILESYSTEM}
    ${CPP_FMT}
    ${AEROTECH_A3200_INCLUDE_DIR}
)

link_directories(
    ${AEROTECH_A3200_LIB_DIR}
)

set(APP_H_SRCS
    App/PSO.h
    App/Options.h
    App/Exception.h
    App/Controller.h
    App/Axis.h
)

set(APP_CPP_SRCS
    App/PSO.cpp
    App/Controller.cpp
    App/Axis.cpp
)

set(APP_SRCS
    ${APP_H_SRCS}
    ${APP_CPP_SRCS}
)

SOURCE_GROUP("App" FILES
    ${APP_SRCS}
)

set(LIBAEROTECH_SRCS
    ${BASE_SRCS}
    ${APP_SRCS}
)

if(BUILD_PYTHON)
    # Add the library
    add_library(aerotech_static STATIC ${LIBAEROTECH_SRCS})

    target_link_libraries(aerotech_static PRIVATE A3200C64)
    GENERATE_EXPORT_HEADER(aerotech_static
                 BASE_NAME AEROTECH
                 EXPORT_MACRO_NAME AEROTECH_EXPORT
                 EXPORT_FILE_NAME AEROTECH_Export.h
                 STATIC_DEFINE AEROTECH_BUILT_AS_STATIC)


else(BUILD_PYTHON)
    message(STATUS "Building pyaerotech Python Module - Dynamic Library")
    add_library(aerotech SHARED ${LIBAEROTECH_SRCS})
    target_link_libraries(aerotech A3200C64)

    GENERATE_EXPORT_HEADER(aerotech
                 BASE_NAME AEROTECH
                 EXPORT_MACRO_NAME AEROTECH_EXPORT
                 EXPORT_FILE_NAME AEROTECH_Export.h
                 STATIC_DEFINE AEROTECH_BUILT_AS_STATIC)
endif(BUILD_PYTHON)

set(App_SRCS
    main.cpp
)

SOURCE_GROUP("App" FILES ${App_SRCS})

#add_subdirectory(Translators)

if(BUILD_PYTHON)
    set(PYAEROTECH_SRCS

        python/pyaerotech/module.cpp
    )

    SOURCE_GROUP("Python" FILES ${PYAEROTECH_SRCS})

    pybind11_add_module(a3200 ${PYAEROTECH_SRCS})

    #add_library(example MODULE main.cpp)
    target_link_libraries(a3200 PRIVATE pybind11::module aerotech_static A3200C64)
    set_target_properties(a3200 PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                           SUFFIX "${PYTHON_MODULE_EXTENSION}")


     add_executable(main ${App_SRCS})
     target_link_libraries(main aerotech_static A3200C64)

    #install(TARGETS slm DESTINATION lib/libSLM)

else()

    add_executable(main ${App_SRCS})
    target_link_libraries(main aerotech A3200C64)

endif(BUILD_PYTHON)

file(GLOB A3200_DLL_LIBS
  "/External/A3200/Bin64/*.dll"
)

file(COPY ${A3200_DLL_LIBS}
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/A3200_Export.h
    ${APP_H_SRCS}
    DESTINATION include/aerotech
)

