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
CMAKE_COMMAND = /snap/clion/82/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/82/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wendy/study/learnSLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wendy/study/learnSLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/learnSLAM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/learnSLAM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/learnSLAM.dir/flags.make

CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o: CMakeFiles/learnSLAM.dir/flags.make
CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o: ../test/test_readImage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wendy/study/learnSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o -c /home/wendy/study/learnSLAM/test/test_readImage.cpp

CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wendy/study/learnSLAM/test/test_readImage.cpp > CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.i

CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wendy/study/learnSLAM/test/test_readImage.cpp -o CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.s

# Object files for target learnSLAM
learnSLAM_OBJECTS = \
"CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o"

# External object files for target learnSLAM
learnSLAM_EXTERNAL_OBJECTS =

../bin/learnSLAM: CMakeFiles/learnSLAM.dir/test/test_readImage.cpp.o
../bin/learnSLAM: CMakeFiles/learnSLAM.dir/build.make
../bin/learnSLAM: CMakeFiles/learnSLAM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wendy/study/learnSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/learnSLAM"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/learnSLAM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/learnSLAM.dir/build: ../bin/learnSLAM

.PHONY : CMakeFiles/learnSLAM.dir/build

CMakeFiles/learnSLAM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/learnSLAM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/learnSLAM.dir/clean

CMakeFiles/learnSLAM.dir/depend:
	cd /home/wendy/study/learnSLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wendy/study/learnSLAM /home/wendy/study/learnSLAM /home/wendy/study/learnSLAM/build /home/wendy/study/learnSLAM/build /home/wendy/study/learnSLAM/build/CMakeFiles/learnSLAM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/learnSLAM.dir/depend
