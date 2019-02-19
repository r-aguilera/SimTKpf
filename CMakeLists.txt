# Generic CMakeLists.txt for making a Simbody-using executable.
# This shows how to use the provided SimbodyConfig.cmake to locate a Simbody
# installation on your machine so you can use it from your own code.
# You will most likely want to copy some of these lines into your own
# CMakeLists.txt rather than use this one verbatim.
cmake_minimum_required(VERSION 2.8)
project(FourBar)

# List your source and header files here.
set(my_source_files Fourbar.cpp)
#set(my_header_files myexe.h)

# This depends on SimbodyConfig.cmake being located somewhere predictable
# on your machine. If you have installed it somewhere that CMake won't be
# able to guess, you'll need to tell find_package where to look.
find_package(Simbody REQUIRED)

# This is only necessary if Simbody was built using CMake older than 3.0.2.
#include_directories(${Simbody_INCLUDE_DIR})

add_executable(FourBar ${my_source_files} )#${my_header_files})
target_link_libraries(FourBar ${Simbody_LIBRARIES})