# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alvise/3DComplete-Project-by-ARMA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alvise/3DComplete-Project-by-ARMA

# Include any dependencies generated for this target.
include CMakeFiles/coplanarity.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/coplanarity.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/coplanarity.dir/flags.make

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o: CMakeFiles/coplanarity.dir/flags.make
CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o: src/coplanarity.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/alvise/3DComplete-Project-by-ARMA/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o -c /home/alvise/3DComplete-Project-by-ARMA/src/coplanarity.cpp

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/coplanarity.dir/src/coplanarity.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/alvise/3DComplete-Project-by-ARMA/src/coplanarity.cpp > CMakeFiles/coplanarity.dir/src/coplanarity.cpp.i

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/coplanarity.dir/src/coplanarity.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/alvise/3DComplete-Project-by-ARMA/src/coplanarity.cpp -o CMakeFiles/coplanarity.dir/src/coplanarity.cpp.s

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.requires:
.PHONY : CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.requires

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.provides: CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.requires
	$(MAKE) -f CMakeFiles/coplanarity.dir/build.make CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.provides.build
.PHONY : CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.provides

CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.provides.build: CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o

# Object files for target coplanarity
coplanarity_OBJECTS = \
"CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o"

# External object files for target coplanarity
coplanarity_EXTERNAL_OBJECTS =

coplanarity: CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o
coplanarity: /usr/lib/libboost_system-mt.so
coplanarity: /usr/lib/libboost_filesystem-mt.so
coplanarity: /usr/lib/libboost_thread-mt.so
coplanarity: /usr/lib/libboost_date_time-mt.so
coplanarity: /usr/lib/libboost_iostreams-mt.so
coplanarity: /usr/lib/libpcl_common.so
coplanarity: /usr/lib/libpcl_octree.so
coplanarity: /usr/lib/libOpenNI.so
coplanarity: /usr/lib/libvtkCommon.so.5.8.0
coplanarity: /usr/lib/libvtkRendering.so.5.8.0
coplanarity: /usr/lib/libvtkHybrid.so.5.8.0
coplanarity: /usr/lib/libpcl_io.so
coplanarity: /usr/lib/libflann_cpp_s.a
coplanarity: /usr/lib/libpcl_kdtree.so
coplanarity: /usr/lib/libpcl_search.so
coplanarity: /usr/lib/libqhull.so
coplanarity: /usr/lib/libpcl_surface.so
coplanarity: /usr/lib/libpcl_sample_consensus.so
coplanarity: /usr/lib/libpcl_filters.so
coplanarity: /usr/lib/libpcl_features.so
coplanarity: /usr/local/lib/libpcl_geometry.so
coplanarity: /usr/lib/libpcl_visualization.so
coplanarity: /usr/local/lib/libpcl_ml.so
coplanarity: /usr/lib/libpcl_segmentation.so
coplanarity: /usr/lib/libpcl_keypoints.so
coplanarity: /usr/lib/libpcl_registration.so
coplanarity: /usr/local/lib/libpcl_recognition.so
coplanarity: /usr/local/lib/libpcl_outofcore.so
coplanarity: /usr/lib/libpcl_tracking.so
coplanarity: /usr/lib/libpcl_apps.so
coplanarity: /usr/local/lib/libpcl_3d_rec_framework.so
coplanarity: /usr/local/lib/libpcl_stereo.so
coplanarity: /usr/lib/libvtkParallel.so.5.8.0
coplanarity: /usr/lib/libvtkRendering.so.5.8.0
coplanarity: /usr/lib/libvtkGraphics.so.5.8.0
coplanarity: /usr/lib/libvtkImaging.so.5.8.0
coplanarity: /usr/lib/libvtkIO.so.5.8.0
coplanarity: /usr/lib/libvtkFiltering.so.5.8.0
coplanarity: /usr/lib/libvtkCommon.so.5.8.0
coplanarity: /usr/lib/libvtksys.so.5.8.0
coplanarity: CMakeFiles/coplanarity.dir/build.make
coplanarity: CMakeFiles/coplanarity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable coplanarity"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/coplanarity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/coplanarity.dir/build: coplanarity
.PHONY : CMakeFiles/coplanarity.dir/build

CMakeFiles/coplanarity.dir/requires: CMakeFiles/coplanarity.dir/src/coplanarity.cpp.o.requires
.PHONY : CMakeFiles/coplanarity.dir/requires

CMakeFiles/coplanarity.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/coplanarity.dir/cmake_clean.cmake
.PHONY : CMakeFiles/coplanarity.dir/clean

CMakeFiles/coplanarity.dir/depend:
	cd /home/alvise/3DComplete-Project-by-ARMA && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alvise/3DComplete-Project-by-ARMA /home/alvise/3DComplete-Project-by-ARMA /home/alvise/3DComplete-Project-by-ARMA /home/alvise/3DComplete-Project-by-ARMA /home/alvise/3DComplete-Project-by-ARMA/CMakeFiles/coplanarity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/coplanarity.dir/depend

