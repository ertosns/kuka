cmake_minimum_required(VERSION 3.2)

set(project_name kuka)
set(test_name kuka_test)

project(${project_name})

# debuging mode
set(CMAKE_BUILD_TYPE Debug)


include_directories(include /usr/include/eigen3)
include_directories(include /usr/include/gtest)
find_library(gtest gtest)
find_library(pthread pthread)
find_library(algebra algebra)
find_library(kinetics kinetics)
# include kuka header
include_directories(include ./kinetics)
include_directories(include ./kinetics/src)
include_directories(include ./kinetics/include)


#######################
# build libkinetics.a #
#######################
add_library(${project_name} STATIC
  ./include/kuka.hpp
  ./include/vehicle.hpp
  ./include/utils.hpp
  ./src/kuka.cpp
  ./src/vehicle.cpp
  ./src/utils.cpp
  )

###############
# build tests #
###############
add_executable(${test_name}
  ./tests/kuka_test.cpp
  )
target_link_libraries(${test_name} gtest pthread algebra kinetics)

##############
# build docs #
##############
execute_process(COMMAND doxygen docs/Doxyfile
  WORKING_DIRECTORY docs
  )

execute_process(COMMAND make
  WORKING_DIRECTORY docs/latex
  )
