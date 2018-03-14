cmake_minimum_required(VERSION 3.9)
project(MAX31865)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_library(MAX31865 SHARED MAX31865.cpp MAX31865.h)
