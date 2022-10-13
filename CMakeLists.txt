cmake_minimum_required(VERSION 3.0)

project(wristdemo)

find_package(YARP REQUIRED)
add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES})

install(TARGETS wristdemo RUNTIME DESTINATION bin)

