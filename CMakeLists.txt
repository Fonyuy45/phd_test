cmake_minimum_required(VERSION 3.22)
project(prog_test)

find_package(Eigen3 REQUIRED)

add_executable(
    ${PROJECT_NAME}
    src/Test.cpp
    src/PathFollower.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE ${cm_interfaces_TARGETS}
            Eigen3::Eigen
)
