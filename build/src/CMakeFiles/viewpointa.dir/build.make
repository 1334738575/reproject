# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/anoorb2/vs_pro/framework

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anoorb2/vs_pro/framework/build

# Include any dependencies generated for this target.
include src/CMakeFiles/viewpointa.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/viewpointa.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/viewpointa.dir/flags.make

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o: src/CMakeFiles/viewpointa.dir/flags.make
src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o: ../src/Virtual_ViewPoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anoorb2/vs_pro/framework/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o"
	cd /home/anoorb2/vs_pro/framework/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o -c /home/anoorb2/vs_pro/framework/src/Virtual_ViewPoint.cpp

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.i"
	cd /home/anoorb2/vs_pro/framework/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anoorb2/vs_pro/framework/src/Virtual_ViewPoint.cpp > CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.i

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.s"
	cd /home/anoorb2/vs_pro/framework/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anoorb2/vs_pro/framework/src/Virtual_ViewPoint.cpp -o CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.s

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.requires:

.PHONY : src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.requires

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.provides: src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/viewpointa.dir/build.make src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.provides.build
.PHONY : src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.provides

src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.provides.build: src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o


# Object files for target viewpointa
viewpointa_OBJECTS = \
"CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o"

# External object files for target viewpointa
viewpointa_EXTERNAL_OBJECTS =

../lib/libviewpointa.so: src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o
../lib/libviewpointa.so: src/CMakeFiles/viewpointa.dir/build.make
../lib/libviewpointa.so: src/CMakeFiles/viewpointa.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anoorb2/vs_pro/framework/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../lib/libviewpointa.so"
	cd /home/anoorb2/vs_pro/framework/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viewpointa.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/viewpointa.dir/build: ../lib/libviewpointa.so

.PHONY : src/CMakeFiles/viewpointa.dir/build

src/CMakeFiles/viewpointa.dir/requires: src/CMakeFiles/viewpointa.dir/Virtual_ViewPoint.cpp.o.requires

.PHONY : src/CMakeFiles/viewpointa.dir/requires

src/CMakeFiles/viewpointa.dir/clean:
	cd /home/anoorb2/vs_pro/framework/build/src && $(CMAKE_COMMAND) -P CMakeFiles/viewpointa.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/viewpointa.dir/clean

src/CMakeFiles/viewpointa.dir/depend:
	cd /home/anoorb2/vs_pro/framework/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anoorb2/vs_pro/framework /home/anoorb2/vs_pro/framework/src /home/anoorb2/vs_pro/framework/build /home/anoorb2/vs_pro/framework/build/src /home/anoorb2/vs_pro/framework/build/src/CMakeFiles/viewpointa.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/viewpointa.dir/depend

