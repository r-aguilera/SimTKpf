# Generic CMakeLists.txt for making a Simbody-using executable.
# This shows how to use the provided SimbodyConfig.cmake to locate a Simbody
# installation on your machine so you can use it from your own code.
# You will most likely want to copy some of these lines into your own
# CMakeLists.txt rather than use this one verbatim.
cmake_minimum_required(VERSION 2.8)
project(SimTKpf)

# List your source and header files here.
set(my_source_files
	src/Fourbar_PtNumberTest.cpp
	src/Gyroscope.cpp
	src/Measuring_Instrument.cpp
	src/Particle_Classes.cpp
	src/PF_utilities.cpp
	
)
set(my_header_files
	include/Gyroscope.h
	include/Measuring_Instrument.h
	include/Particle_Classes.h
	include/PF_utilities.h
)

# This depends on SimbodyConfig.cmake being located somewhere predictable
# on your machine. If you have installed it somewhere that CMake won't be
# able to guess, you'll need to tell find_package where to look.
find_package(Simbody REQUIRED)

# This is only necessary if Simbody was built using CMake older than 3.0.2.
#include_directories(${Simbody_INCLUDE_DIR})

add_library(SimTKpf ${my_source_files} ${my_header_files})
target_link_libraries(SimTKpf PUBLIC ${Simbody_LIBRARIES})
target_include_directories(SimTKpf PUBLIC "${CMAKE_SOURCE_DIR}/include")

add_executable(Fourbar_PtNumberTest ${my_source_files} )#${my_header_files})
target_link_libraries(Fourbar_PtNumberTest SimTKpf)
