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
include kinetics/CMakeFiles/kinetics.dir/depend.make

# Include the progress variables for this target.
include kinetics/CMakeFiles/kinetics.dir/progress.make

# Include the compile flags for this target's objects.
include kinetics/CMakeFiles/kinetics.dir/flags.make

kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.o: kinetics/CMakeFiles/kinetics.dir/flags.make
kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.o: ../kinetics/src/kinetics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.o"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinetics.dir/src/kinetics.cpp.o -c /home/zerin/prj/kuka/kinetics/src/kinetics.cpp

kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinetics.dir/src/kinetics.cpp.i"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zerin/prj/kuka/kinetics/src/kinetics.cpp > CMakeFiles/kinetics.dir/src/kinetics.cpp.i

kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinetics.dir/src/kinetics.cpp.s"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zerin/prj/kuka/kinetics/src/kinetics.cpp -o CMakeFiles/kinetics.dir/src/kinetics.cpp.s

kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.o: kinetics/CMakeFiles/kinetics.dir/flags.make
kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.o: ../kinetics/src/kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.o"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinetics.dir/src/kinematics.cpp.o -c /home/zerin/prj/kuka/kinetics/src/kinematics.cpp

kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinetics.dir/src/kinematics.cpp.i"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zerin/prj/kuka/kinetics/src/kinematics.cpp > CMakeFiles/kinetics.dir/src/kinematics.cpp.i

kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinetics.dir/src/kinematics.cpp.s"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zerin/prj/kuka/kinetics/src/kinematics.cpp -o CMakeFiles/kinetics.dir/src/kinematics.cpp.s

kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.o: kinetics/CMakeFiles/kinetics.dir/flags.make
kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.o: ../kinetics/src/trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.o"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinetics.dir/src/trajectory.cpp.o -c /home/zerin/prj/kuka/kinetics/src/trajectory.cpp

kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinetics.dir/src/trajectory.cpp.i"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zerin/prj/kuka/kinetics/src/trajectory.cpp > CMakeFiles/kinetics.dir/src/trajectory.cpp.i

kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinetics.dir/src/trajectory.cpp.s"
	cd /home/zerin/prj/kuka/build/kinetics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zerin/prj/kuka/kinetics/src/trajectory.cpp -o CMakeFiles/kinetics.dir/src/trajectory.cpp.s

# Object files for target kinetics
kinetics_OBJECTS = \
"CMakeFiles/kinetics.dir/src/kinetics.cpp.o" \
"CMakeFiles/kinetics.dir/src/kinematics.cpp.o" \
"CMakeFiles/kinetics.dir/src/trajectory.cpp.o"

# External object files for target kinetics
kinetics_EXTERNAL_OBJECTS =

kinetics/libkinetics.a: kinetics/CMakeFiles/kinetics.dir/src/kinetics.cpp.o
kinetics/libkinetics.a: kinetics/CMakeFiles/kinetics.dir/src/kinematics.cpp.o
kinetics/libkinetics.a: kinetics/CMakeFiles/kinetics.dir/src/trajectory.cpp.o
kinetics/libkinetics.a: kinetics/CMakeFiles/kinetics.dir/build.make
kinetics/libkinetics.a: kinetics/CMakeFiles/kinetics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zerin/prj/kuka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libkinetics.a"
	cd /home/zerin/prj/kuka/build/kinetics && $(CMAKE_COMMAND) -P CMakeFiles/kinetics.dir/cmake_clean_target.cmake
	cd /home/zerin/prj/kuka/build/kinetics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinetics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kinetics/CMakeFiles/kinetics.dir/build: kinetics/libkinetics.a

.PHONY : kinetics/CMakeFiles/kinetics.dir/build

kinetics/CMakeFiles/kinetics.dir/clean:
	cd /home/zerin/prj/kuka/build/kinetics && $(CMAKE_COMMAND) -P CMakeFiles/kinetics.dir/cmake_clean.cmake
.PHONY : kinetics/CMakeFiles/kinetics.dir/clean

kinetics/CMakeFiles/kinetics.dir/depend:
	cd /home/zerin/prj/kuka/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zerin/prj/kuka /home/zerin/prj/kuka/kinetics /home/zerin/prj/kuka/build /home/zerin/prj/kuka/build/kinetics /home/zerin/prj/kuka/build/kinetics/CMakeFiles/kinetics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kinetics/CMakeFiles/kinetics.dir/depend

