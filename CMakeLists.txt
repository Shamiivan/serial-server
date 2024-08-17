cmake_minimum_required(VERSION 3.12)
project(serial-server)

set(CMAKE_CXX_STANDARD 20)

# Add your source files here
set(SOURCES
    src/main.cpp
    src/serial.cpp
)

# Add any additional include directories here
include_directories(include)

# Add any additional libraries here
# find_package(<library_name> REQUIRED)

add_executable(serial-server ${SOURCES})
# target_link_libraries(serial-server <library_name>)