# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty

# Include any dependencies generated for this target.
include CMakeFiles/cubemap_fangshan.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cubemap_fangshan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cubemap_fangshan.dir/flags.make

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o: CMakeFiles/cubemap_fangshan.dir/flags.make
CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o: ../Examples/cubemap_fangshan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o -c /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/Examples/cubemap_fangshan.cpp

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/Examples/cubemap_fangshan.cpp > CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.i

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/Examples/cubemap_fangshan.cpp -o CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.s

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.requires:

.PHONY : CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.requires

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.provides: CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.requires
	$(MAKE) -f CMakeFiles/cubemap_fangshan.dir/build.make CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.provides.build
.PHONY : CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.provides

CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.provides.build: CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o


# Object files for target cubemap_fangshan
cubemap_fangshan_OBJECTS = \
"CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o"

# External object files for target cubemap_fangshan
cubemap_fangshan_EXTERNAL_OBJECTS =

../bin/cubemap_fangshan: CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o
../bin/cubemap_fangshan: CMakeFiles/cubemap_fangshan.dir/build.make
../bin/cubemap_fangshan: ../lib/libCubemapSLAM.so
../bin/cubemap_fangshan: /usr/local/lib/libopencv_dnn.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_gapi.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_highgui.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_ml.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_objdetect.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_photo.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_stitching.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_video.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_calib3d.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_features2d.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_flann.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_videoio.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_imgproc.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libopencv_core.so.4.4.0
../bin/cubemap_fangshan: /usr/local/lib/libpangolin.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libavcodec.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libavformat.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libavutil.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libswscale.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libpng.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libz.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libjpeg.so
../bin/cubemap_fangshan: /usr/lib/x86_64-linux-gnu/libtiff.so
../bin/cubemap_fangshan: g2o/lib/libg2o.so
../bin/cubemap_fangshan: DBoW2/lib/libDBoW2.so
../bin/cubemap_fangshan: CMakeFiles/cubemap_fangshan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/cubemap_fangshan"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cubemap_fangshan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cubemap_fangshan.dir/build: ../bin/cubemap_fangshan

.PHONY : CMakeFiles/cubemap_fangshan.dir/build

CMakeFiles/cubemap_fangshan.dir/requires: CMakeFiles/cubemap_fangshan.dir/Examples/cubemap_fangshan.cpp.o.requires

.PHONY : CMakeFiles/cubemap_fangshan.dir/requires

CMakeFiles/cubemap_fangshan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cubemap_fangshan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cubemap_fangshan.dir/clean

CMakeFiles/cubemap_fangshan.dir/depend:
	cd /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty /home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/code/CubemapSLAM/ThirdParty/CMakeFiles/cubemap_fangshan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cubemap_fangshan.dir/depend
