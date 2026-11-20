cmake_minimum_required(VERSION 3.19)
project(control_core LANGUAGES CXX)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
                OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
                COMMAND_ERROR_IS_FATAL ANY)
find_package(pybind11 CONFIG REQUIRED)
find_package(Threads REQUIRED)
pybind11_add_module(_native src/controller.cpp)
target_compile_features(_native PRIVATE cxx_std_17)
target_link_libraries(_native PRIVATE Threads::Threads)
install(TARGETS _native LIBRARY DESTINATION control_core)
