# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/silence401/wujinbo/cmake-3.9.1-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/silence401/wujinbo/cmake-3.9.1-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/silence401/wujinbo/rayinsection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/silence401/wujinbo/rayinsection/build

# Include any dependencies generated for this target.
include CMakeFiles/raycast.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/raycast.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/raycast.dir/flags.make

CMakeFiles/raycast.dir/alpha_expansion.cpp.o: CMakeFiles/raycast.dir/flags.make
CMakeFiles/raycast.dir/alpha_expansion.cpp.o: ../alpha_expansion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/silence401/wujinbo/rayinsection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/raycast.dir/alpha_expansion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/raycast.dir/alpha_expansion.cpp.o -c /home/silence401/wujinbo/rayinsection/alpha_expansion.cpp

CMakeFiles/raycast.dir/alpha_expansion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/raycast.dir/alpha_expansion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/silence401/wujinbo/rayinsection/alpha_expansion.cpp > CMakeFiles/raycast.dir/alpha_expansion.cpp.i

CMakeFiles/raycast.dir/alpha_expansion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/raycast.dir/alpha_expansion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/silence401/wujinbo/rayinsection/alpha_expansion.cpp -o CMakeFiles/raycast.dir/alpha_expansion.cpp.s

CMakeFiles/raycast.dir/alpha_expansion.cpp.o.requires:

.PHONY : CMakeFiles/raycast.dir/alpha_expansion.cpp.o.requires

CMakeFiles/raycast.dir/alpha_expansion.cpp.o.provides: CMakeFiles/raycast.dir/alpha_expansion.cpp.o.requires
	$(MAKE) -f CMakeFiles/raycast.dir/build.make CMakeFiles/raycast.dir/alpha_expansion.cpp.o.provides.build
.PHONY : CMakeFiles/raycast.dir/alpha_expansion.cpp.o.provides

CMakeFiles/raycast.dir/alpha_expansion.cpp.o.provides.build: CMakeFiles/raycast.dir/alpha_expansion.cpp.o


CMakeFiles/raycast.dir/main.cpp.o: CMakeFiles/raycast.dir/flags.make
CMakeFiles/raycast.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/silence401/wujinbo/rayinsection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/raycast.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/raycast.dir/main.cpp.o -c /home/silence401/wujinbo/rayinsection/main.cpp

CMakeFiles/raycast.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/raycast.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/silence401/wujinbo/rayinsection/main.cpp > CMakeFiles/raycast.dir/main.cpp.i

CMakeFiles/raycast.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/raycast.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/silence401/wujinbo/rayinsection/main.cpp -o CMakeFiles/raycast.dir/main.cpp.s

CMakeFiles/raycast.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/raycast.dir/main.cpp.o.requires

CMakeFiles/raycast.dir/main.cpp.o.provides: CMakeFiles/raycast.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/raycast.dir/build.make CMakeFiles/raycast.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/raycast.dir/main.cpp.o.provides

CMakeFiles/raycast.dir/main.cpp.o.provides.build: CMakeFiles/raycast.dir/main.cpp.o


# Object files for target raycast
raycast_OBJECTS = \
"CMakeFiles/raycast.dir/alpha_expansion.cpp.o" \
"CMakeFiles/raycast.dir/main.cpp.o"

# External object files for target raycast
raycast_EXTERNAL_OBJECTS =

raycast: CMakeFiles/raycast.dir/alpha_expansion.cpp.o
raycast: CMakeFiles/raycast.dir/main.cpp.o
raycast: CMakeFiles/raycast.dir/build.make
raycast: maxflow/libmaxflow.a
raycast: /usr/lib/x86_64-linux-gnu/libjpeg.so
raycast: /usr/lib/x86_64-linux-gnu/libpng.so
raycast: /usr/lib/x86_64-linux-gnu/libz.so
raycast: /usr/lib/x86_64-linux-gnu/libtiff.so
raycast: /usr/local/lib/libopencv_dnn.so.4.4.0
raycast: /usr/local/lib/libopencv_gapi.so.4.4.0
raycast: /usr/local/lib/libopencv_highgui.so.4.4.0
raycast: /usr/local/lib/libopencv_ml.so.4.4.0
raycast: /usr/local/lib/libopencv_objdetect.so.4.4.0
raycast: /usr/local/lib/libopencv_photo.so.4.4.0
raycast: /usr/local/lib/libopencv_stitching.so.4.4.0
raycast: /usr/local/lib/libopencv_video.so.4.4.0
raycast: /usr/local/lib/libopencv_videoio.so.4.4.0
raycast: /usr/lib/x86_64-linux-gnu/libGL.so
raycast: glad/libglad.a
raycast: glfw/src/libglfw3.a
raycast: /usr/lib/x86_64-linux-gnu/librt.so
raycast: /usr/lib/x86_64-linux-gnu/libm.so
raycast: /usr/lib/x86_64-linux-gnu/libX11.so
raycast: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
raycast: /usr/local/lib/libopencv_calib3d.so.4.4.0
raycast: /usr/local/lib/libopencv_features2d.so.4.4.0
raycast: /usr/local/lib/libopencv_flann.so.4.4.0
raycast: /usr/local/lib/libopencv_imgproc.so.4.4.0
raycast: /usr/local/lib/libopencv_core.so.4.4.0
raycast: CMakeFiles/raycast.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/silence401/wujinbo/rayinsection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable raycast"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/raycast.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/raycast.dir/build: raycast

.PHONY : CMakeFiles/raycast.dir/build

CMakeFiles/raycast.dir/requires: CMakeFiles/raycast.dir/alpha_expansion.cpp.o.requires
CMakeFiles/raycast.dir/requires: CMakeFiles/raycast.dir/main.cpp.o.requires

.PHONY : CMakeFiles/raycast.dir/requires

CMakeFiles/raycast.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/raycast.dir/cmake_clean.cmake
.PHONY : CMakeFiles/raycast.dir/clean

CMakeFiles/raycast.dir/depend:
	cd /home/silence401/wujinbo/rayinsection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/silence401/wujinbo/rayinsection /home/silence401/wujinbo/rayinsection /home/silence401/wujinbo/rayinsection/build /home/silence401/wujinbo/rayinsection/build /home/silence401/wujinbo/rayinsection/build/CMakeFiles/raycast.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/raycast.dir/depend
