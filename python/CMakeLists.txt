cmake_minimum_required(VERSION 3.4)
project(libfranka-python LANGUAGES CXX)
find_package(Python 3.9 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)


list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)
set(Python_ROOT_DIR "/home/aidara/miniconda3/envs/act/bin/python")


find_package(pybind11 REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

pybind11_add_module(franka_py bindings.cpp examples_common.cpp)

target_link_libraries(franka_py
  PRIVATE Franka::Franka
          Eigen3::Eigen3
)


