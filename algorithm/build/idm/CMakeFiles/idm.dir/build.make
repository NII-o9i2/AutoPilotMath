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
CMAKE_SOURCE_DIR = /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build

# Include any dependencies generated for this target.
include idm/CMakeFiles/idm.dir/depend.make

# Include the progress variables for this target.
include idm/CMakeFiles/idm.dir/progress.make

# Include the compile flags for this target's objects.
include idm/CMakeFiles/idm.dir/flags.make

idm/CMakeFiles/idm.dir/idm.cpp.o: idm/CMakeFiles/idm.dir/flags.make
idm/CMakeFiles/idm.dir/idm.cpp.o: ../idm/idm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object idm/CMakeFiles/idm.dir/idm.cpp.o"
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/idm.dir/idm.cpp.o -c /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/idm/idm.cpp

idm/CMakeFiles/idm.dir/idm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/idm.dir/idm.cpp.i"
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/idm/idm.cpp > CMakeFiles/idm.dir/idm.cpp.i

idm/CMakeFiles/idm.dir/idm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/idm.dir/idm.cpp.s"
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/idm/idm.cpp -o CMakeFiles/idm.dir/idm.cpp.s

# Object files for target idm
idm_OBJECTS = \
"CMakeFiles/idm.dir/idm.cpp.o"

# External object files for target idm
idm_EXTERNAL_OBJECTS =

idm/libidm.so: idm/CMakeFiles/idm.dir/idm.cpp.o
idm/libidm.so: idm/CMakeFiles/idm.dir/build.make
idm/libidm.so: idm/CMakeFiles/idm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libidm.so"
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/idm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
idm/CMakeFiles/idm.dir/build: idm/libidm.so

.PHONY : idm/CMakeFiles/idm.dir/build

idm/CMakeFiles/idm.dir/clean:
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm && $(CMAKE_COMMAND) -P CMakeFiles/idm.dir/cmake_clean.cmake
.PHONY : idm/CMakeFiles/idm.dir/clean

idm/CMakeFiles/idm.dir/depend:
	cd /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/idm /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm /home/SENSETIME/fengxiaotong/ws/rscl_aarch64/senseauto-pilot-decision/common_math/algorithm/build/idm/CMakeFiles/idm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : idm/CMakeFiles/idm.dir/depend

