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
CMAKE_SOURCE_DIR = /home/nim/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nim/catkin_ws/build

# Include any dependencies generated for this target.
include bebo_control/CMakeFiles/bebop_test.dir/depend.make

# Include the progress variables for this target.
include bebo_control/CMakeFiles/bebop_test.dir/progress.make

# Include the compile flags for this target's objects.
include bebo_control/CMakeFiles/bebop_test.dir/flags.make

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o: bebo_control/CMakeFiles/bebop_test.dir/flags.make
bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o: /home/nim/catkin_ws/src/bebo_control/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nim/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bebop_test.dir/src/test.cpp.o -c /home/nim/catkin_ws/src/bebo_control/src/test.cpp

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bebop_test.dir/src/test.cpp.i"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nim/catkin_ws/src/bebo_control/src/test.cpp > CMakeFiles/bebop_test.dir/src/test.cpp.i

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bebop_test.dir/src/test.cpp.s"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nim/catkin_ws/src/bebo_control/src/test.cpp -o CMakeFiles/bebop_test.dir/src/test.cpp.s

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.requires:

.PHONY : bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.requires

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.provides: bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.requires
	$(MAKE) -f bebo_control/CMakeFiles/bebop_test.dir/build.make bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.provides.build
.PHONY : bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.provides

bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.provides.build: bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o


bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o: bebo_control/CMakeFiles/bebop_test.dir/flags.make
bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o: /home/nim/catkin_ws/src/bebo_control/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nim/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bebop_test.dir/src/main.cpp.o -c /home/nim/catkin_ws/src/bebo_control/src/main.cpp

bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bebop_test.dir/src/main.cpp.i"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nim/catkin_ws/src/bebo_control/src/main.cpp > CMakeFiles/bebop_test.dir/src/main.cpp.i

bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bebop_test.dir/src/main.cpp.s"
	cd /home/nim/catkin_ws/build/bebo_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nim/catkin_ws/src/bebo_control/src/main.cpp -o CMakeFiles/bebop_test.dir/src/main.cpp.s

bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.requires:

.PHONY : bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.requires

bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.provides: bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.requires
	$(MAKE) -f bebo_control/CMakeFiles/bebop_test.dir/build.make bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.provides.build
.PHONY : bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.provides

bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.provides.build: bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o


# Object files for target bebop_test
bebop_test_OBJECTS = \
"CMakeFiles/bebop_test.dir/src/test.cpp.o" \
"CMakeFiles/bebop_test.dir/src/main.cpp.o"

# External object files for target bebop_test
bebop_test_EXTERNAL_OBJECTS =

/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: bebo_control/CMakeFiles/bebop_test.dir/build.make
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/libroscpp.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/librosconsole.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/librostime.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /opt/ros/kinetic/lib/libcpp_common.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/nim/catkin_ws/devel/lib/bebop_control/bebop_test: bebo_control/CMakeFiles/bebop_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nim/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/nim/catkin_ws/devel/lib/bebop_control/bebop_test"
	cd /home/nim/catkin_ws/build/bebo_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bebop_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bebo_control/CMakeFiles/bebop_test.dir/build: /home/nim/catkin_ws/devel/lib/bebop_control/bebop_test

.PHONY : bebo_control/CMakeFiles/bebop_test.dir/build

bebo_control/CMakeFiles/bebop_test.dir/requires: bebo_control/CMakeFiles/bebop_test.dir/src/test.cpp.o.requires
bebo_control/CMakeFiles/bebop_test.dir/requires: bebo_control/CMakeFiles/bebop_test.dir/src/main.cpp.o.requires

.PHONY : bebo_control/CMakeFiles/bebop_test.dir/requires

bebo_control/CMakeFiles/bebop_test.dir/clean:
	cd /home/nim/catkin_ws/build/bebo_control && $(CMAKE_COMMAND) -P CMakeFiles/bebop_test.dir/cmake_clean.cmake
.PHONY : bebo_control/CMakeFiles/bebop_test.dir/clean

bebo_control/CMakeFiles/bebop_test.dir/depend:
	cd /home/nim/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nim/catkin_ws/src /home/nim/catkin_ws/src/bebo_control /home/nim/catkin_ws/build /home/nim/catkin_ws/build/bebo_control /home/nim/catkin_ws/build/bebo_control/CMakeFiles/bebop_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bebo_control/CMakeFiles/bebop_test.dir/depend

