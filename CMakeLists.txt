cmake_minimum_required(VERSION 3.12)
project(SoftRobotsSU VERSION 1.0 LANGUAGES CXX)

# Find and load CMake configuration of packages containing this plugin's dependencies
## Mandatory dependencies
find_package(SofaBase REQUIRED) # Dependency to SofaBaseVisual
## Optional dependencies
sofa_find_package(SofaGui) # Dependency to SofaGuiQt
sofa_find_package(Qt5 COMPONENTS Core) # Dependency to Qt5Core (needed for qt5_wrap_cpp)

# List all files
set(SOFTROBOTSSU_SRC_DIR src/SoftRobotsSU)
set(HEADER_FILES
    ${SOFTROBOTSSU_SRC_DIR}/config.h.in
    ${SOFTROBOTSSU_SRC_DIR}/mapping/RotationAndBarycentricMapping.h
    ${SOFTROBOTSSU_SRC_DIR}/mapping/RotationAndBarycentricMapping.inl
    )
set(HEADER_FILES_TO_MOC
    )
set(SOURCE_FILES
    ${SOFTROBOTSSU_SRC_DIR}/initSoftRobotsSU.cpp
    ${SOFTROBOTSSU_SRC_DIR}/mapping/RotationAndBarycentricMapping.cpp
    )
set(README_FILES
    SoftRobotsSU.md
    )

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${MOCCED_HEADER_FILES} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Link the plugin library to its dependency(ies).
target_link_libraries(${PROJECT_NAME} SofaBaseVisual SofaMiscMapping)
# Link with the optional GUI dependencies.
if(Qt5Core_FOUND AND SofaGui_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaGuiQt)
endif()

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
# Set the plugin 'relocatable' if built within SOFA.
# --> see SofaMacros.cmake
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
#cmake_dependent_option(SOFTROBOTSSU_BUILD_TESTS "Compile the automatic tests" ON "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
#if(SOFTROBOTSSU_BUILD_TESTS)
#    enable_testing()
#    add_subdirectory(SoftRobotsSU_test)
#endif()
