# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zerin/prj/kuka

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zerin/prj/kuka/build

# Include any dependencies generated for this target.
include kinetics/algebra/CMakeFiles/algebra_test.dir/depend.make

# Include the progress variables for this target.
include kinetics/algebra/CMakeFiles/algebra_test.dir/progress.make

# Include the compile flags for this target's objects.
include kinetics/algebra/CMakeFiles/algebra_test.dir/flags.make

kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o: kinetics/algebra/CMakeFiles/algebra_test.dir/flags.make
kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o: ../kinetics/algebra/tests/algebra_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o"
	cd /home/zerin/prj/kuka/build/kinetics/algebra && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o -c /home/zerin/prj/kuka/kinetics/algebra/tests/algebra_test.cpp

kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.i"
	cd /home/zerin/prj/kuka/build/kinetics/algebra && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zerin/prj/kuka/kinetics/algebra/tests/algebra_test.cpp > CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.i

kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.s"
	cd /home/zerin/prj/kuka/build/kinetics/algebra && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zerin/prj/kuka/kinetics/algebra/tests/algebra_test.cpp -o CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.s

# Object files for target algebra_test
algebra_test_OBJECTS = \
"CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o"

# External object files for target algebra_test
algebra_test_EXTERNAL_OBJECTS =

kinetics/algebra/algebra_test: kinetics/algebra/CMakeFiles/algebra_test.dir/tests/algebra_test.cpp.o
kinetics/algebra/algebra_test: kinetics/algebra/CMakeFiles/algebra_test.dir/build.make
kinetics/algebra/algebra_test: kinetics/algebra/libalgebra.a
kinetics/algebra/algebra_test: kinetics/algebra/CMakeFiles/algebra_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable algebra_test"
	cd /home/zerin/prj/kuka/build/kinetics/algebra && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/algebra_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kinetics/algebra/CMakeFiles/algebra_test.dir/build: kinetics/algebra/algebra_test

.PHONY : kinetics/algebra/CMakeFiles/algebra_test.dir/build

kinetics/algebra/CMakeFiles/algebra_test.dir/clean:
	cd /home/zerin/prj/kuka/build/kinetics/algebra && $(CMAKE_COMMAND) -P CMakeFiles/algebra_test.dir/cmake_clean.cmake
.PHONY : kinetics/algebra/CMakeFiles/algebra_test.dir/clean

kinetics/algebra/CMakeFiles/algebra_test.dir/depend:
	cd /home/zerin/prj/kuka/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zerin/prj/kuka /home/zerin/prj/kuka/kinetics/algebra /home/zerin/prj/kuka/build /home/zerin/prj/kuka/build/kinetics/algebra /home/zerin/prj/kuka/build/kinetics/algebra/CMakeFiles/algebra_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kinetics/algebra/CMakeFiles/algebra_test.dir/depend

