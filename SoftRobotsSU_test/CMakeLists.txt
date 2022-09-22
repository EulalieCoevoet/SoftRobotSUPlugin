cmake_minimum_required(VERSION 3.12)

project(SoftRobotsSU_test)

find_package(SoftRobotsSU REQUIRED)

set(SOURCE_FILES
    )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaGTestMain SoftRobotsSU)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
