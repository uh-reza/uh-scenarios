# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/reza/ros/image_pipeline/camera_calibration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/ros/image_pipeline/camera_calibration/build

# Utility rule file for pyunit_test_testapproxsync.py.

CMakeFiles/pyunit_test_testapproxsync.py: ../test/testapproxsync.py
	cd /home/reza/ros/image_pipeline/camera_calibration && /home/reza/ros/ros/tools/rosunit/bin/rosunit --name=test_testapproxsync.py --time-limit=60.0 -- test/testapproxsync.py

pyunit_test_testapproxsync.py: CMakeFiles/pyunit_test_testapproxsync.py
pyunit_test_testapproxsync.py: CMakeFiles/pyunit_test_testapproxsync.py.dir/build.make
.PHONY : pyunit_test_testapproxsync.py

# Rule to build all files generated by this target.
CMakeFiles/pyunit_test_testapproxsync.py.dir/build: pyunit_test_testapproxsync.py
.PHONY : CMakeFiles/pyunit_test_testapproxsync.py.dir/build

CMakeFiles/pyunit_test_testapproxsync.py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pyunit_test_testapproxsync.py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pyunit_test_testapproxsync.py.dir/clean

CMakeFiles/pyunit_test_testapproxsync.py.dir/depend:
	cd /home/reza/ros/image_pipeline/camera_calibration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/ros/image_pipeline/camera_calibration /home/reza/ros/image_pipeline/camera_calibration /home/reza/ros/image_pipeline/camera_calibration/build /home/reza/ros/image_pipeline/camera_calibration/build /home/reza/ros/image_pipeline/camera_calibration/build/CMakeFiles/pyunit_test_testapproxsync.py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pyunit_test_testapproxsync.py.dir/depend
