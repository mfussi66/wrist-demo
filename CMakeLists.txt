cmake_minimum_required(VERSION 3.0)

project(wristdemo)

add_executable(wristdemo main.cpp)

install(TARGETS wristdemo RUNTIME DESTINATION bin)
