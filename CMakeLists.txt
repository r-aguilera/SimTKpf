# Generic CMakeLists.txt for making a Simbody-using executable.
# This shows how to use the provided SimbodyConfig.cmake to locate a Simbody
# installation on your machine so you can use it from your own code.
# You will most likely want to copy some of these lines into your own
# CMakeLists.txt rather than use this one verbatim.
cmake_minimum_required(VERSION 2.8)
project(SimTKpf)

# List your source and header files here.
set(my_source_files
	src/Fourbar/Gyroscope.cpp
	src/Fourbar/Txt_write.cpp
	src/SimTKpf/Measuring_Instrument.cpp
	src/SimTKpf/Particle_Classes.cpp
	src/SimTKpf/Particle_Filter.cpp
	src/SimTKpf/PF_utilities.cpp
	src/SimTKpf/Simbody_Instrument.cpp
	src/SimTKpf/Stopwatch.cpp
)

set(my_header_files
	include/Fourbar/Gyroscope.h
	include/Fourbar/Txt_write.h
	include/SimTKpf.h
	include/SimTKpf/Measuring_Instrument.h
	include/SimTKpf/Particle_Classes.h
	include/SimTKpf/Particle_Filter.h
	include/SimTKpf/PF_utilities.h
	include/SimTKpf/Simbody_Instrument.h
	include/SimTKpf/Stopwatch.h
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

add_executable(Fourbar src/Fourbar/Fourbar.cpp)
target_link_libraries(Fourbar SimTKpf)
add_executable(Fourbar_StdDevTest src/Fourbar/Fourbar_StdDevTest.cpp)
target_link_libraries(Fourbar_StdDevTest SimTKpf)
add_executable(Fourbar_PtNumberTest src/Fourbar/Fourbar_PtNumberTest.cpp)
target_link_libraries(Fourbar_PtNumberTest SimTKpf)
