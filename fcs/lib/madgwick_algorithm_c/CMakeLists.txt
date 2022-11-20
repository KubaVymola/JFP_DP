cmake_minimum_required(VERSION 3.13)

project(madgwick_algo VERSION 1.0)

add_library(madgwick_algo STATIC MadgwickAHRS.cpp)
add_library(madgwick_algo::madgwick_algo ALIAS madgwick_algo)
