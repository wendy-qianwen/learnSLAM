# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /snap/clion/83/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/83/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wendy/study/learnSLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wendy/study/learnSLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/readData.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/readData.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/readData.dir/flags.make

CMakeFiles/readData.dir/test/test_readData.cpp.o: CMakeFiles/readData.dir/flags.make
CMakeFiles/readData.dir/test/test_readData.cpp.o: ../test/test_readData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wendy/study/learnSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/readData.dir/test/test_readData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readData.dir/test/test_readData.cpp.o -c /home/wendy/study/learnSLAM/test/test_readData.cpp

CMakeFiles/readData.dir/test/test_readData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readData.dir/test/test_readData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wendy/study/learnSLAM/test/test_readData.cpp > CMakeFiles/readData.dir/test/test_readData.cpp.i

CMakeFiles/readData.dir/test/test_readData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readData.dir/test/test_readData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wendy/study/learnSLAM/test/test_readData.cpp -o CMakeFiles/readData.dir/test/test_readData.cpp.s

# Object files for target readData
readData_OBJECTS = \
"CMakeFiles/readData.dir/test/test_readData.cpp.o"

# External object files for target readData
readData_EXTERNAL_OBJECTS =

../bin/readData: CMakeFiles/readData.dir/test/test_readData.cpp.o
../bin/readData: CMakeFiles/readData.dir/build.make
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/readData: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
../bin/readData: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
../bin/readData: CMakeFiles/readData.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wendy/study/learnSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/readData"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/readData.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/readData.dir/build: ../bin/readData

.PHONY : CMakeFiles/readData.dir/build

CMakeFiles/readData.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/readData.dir/cmake_clean.cmake
.PHONY : CMakeFiles/readData.dir/clean

CMakeFiles/readData.dir/depend:
	cd /home/wendy/study/learnSLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wendy/study/learnSLAM /home/wendy/study/learnSLAM /home/wendy/study/learnSLAM/build /home/wendy/study/learnSLAM/build /home/wendy/study/learnSLAM/build/CMakeFiles/readData.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/readData.dir/depend

