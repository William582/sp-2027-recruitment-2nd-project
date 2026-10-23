cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME tripod_head)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
     Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/can/can_recv.cpp
    applications/can/can_send.cpp
    applications/io/led_task.cpp
    applications/io/buzzer_task.cpp
    applications/controllers/gimbal_folding_task.cpp
    applications/controllers/gimbal_tracking_task.cpp
    applications/controllers/IR_ranging_task.cpp
    applications/plot_task.cpp
    sp_middleware/io/led/led.cpp
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/io/plotter/plotter.cpp
    sp_middleware/io/dbus/dbus.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/crc/crc.cpp
    applications/trajectory_task.cpp
    applications/refer/refer.cpp
    sp_middleware/io/laser/laser.cpp# # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
     applications/
     sp_middleware/# Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
