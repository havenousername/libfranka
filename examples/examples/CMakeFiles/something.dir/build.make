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
CMAKE_SOURCE_DIR = /home/panda/libfranka

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/panda/libfranka/examples

# Include any dependencies generated for this target.
include examples/CMakeFiles/something.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/something.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/something.dir/flags.make

examples/CMakeFiles/something.dir/something.cpp.o: examples/CMakeFiles/something.dir/flags.make
examples/CMakeFiles/something.dir/something.cpp.o: something.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/panda/libfranka/examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/something.dir/something.cpp.o"
	cd /home/panda/libfranka/examples/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/something.dir/something.cpp.o -c /home/panda/libfranka/examples/something.cpp

examples/CMakeFiles/something.dir/something.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/something.dir/something.cpp.i"
	cd /home/panda/libfranka/examples/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/panda/libfranka/examples/something.cpp > CMakeFiles/something.dir/something.cpp.i

examples/CMakeFiles/something.dir/something.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/something.dir/something.cpp.s"
	cd /home/panda/libfranka/examples/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/panda/libfranka/examples/something.cpp -o CMakeFiles/something.dir/something.cpp.s

# Object files for target something
something_OBJECTS = \
"CMakeFiles/something.dir/something.cpp.o"

# External object files for target something
something_EXTERNAL_OBJECTS =

examples/something: examples/CMakeFiles/something.dir/something.cpp.o
examples/something: examples/CMakeFiles/something.dir/build.make
examples/something: examples/libexamples_common.a
examples/something: libfranka.so.0.9.2
examples/something: examples/CMakeFiles/something.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/panda/libfranka/examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable something"
	cd /home/panda/libfranka/examples/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/something.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/something.dir/build: examples/something

.PHONY : examples/CMakeFiles/something.dir/build

examples/CMakeFiles/something.dir/clean:
	cd /home/panda/libfranka/examples/examples && $(CMAKE_COMMAND) -P CMakeFiles/something.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/something.dir/clean

examples/CMakeFiles/something.dir/depend:
	cd /home/panda/libfranka/examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/panda/libfranka /home/panda/libfranka/examples /home/panda/libfranka/examples /home/panda/libfranka/examples/examples /home/panda/libfranka/examples/examples/CMakeFiles/something.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/something.dir/depend

