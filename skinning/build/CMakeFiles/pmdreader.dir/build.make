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
CMAKE_SOURCE_DIR = /home/remusw/Documents/Graphics/Proj4/skinning/skinning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build

# Include any dependencies generated for this target.
include CMakeFiles/pmdreader.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pmdreader.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pmdreader.dir/flags.make

CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o: CMakeFiles/pmdreader.dir/flags.make
CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o: ../lib/pmdreader/bitmap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/bitmap.cpp

CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/bitmap.cpp > CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.i

CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/bitmap.cpp -o CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.s

CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o: CMakeFiles/pmdreader.dir/flags.make
CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o: ../lib/pmdreader/mmdadapter.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/mmdadapter.cc

CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/mmdadapter.cc > CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.i

CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/lib/pmdreader/mmdadapter.cc -o CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.s

# Object files for target pmdreader
pmdreader_OBJECTS = \
"CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o" \
"CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o"

# External object files for target pmdreader
pmdreader_EXTERNAL_OBJECTS =

libpmdreader.a: CMakeFiles/pmdreader.dir/lib/pmdreader/bitmap.cpp.o
libpmdreader.a: CMakeFiles/pmdreader.dir/lib/pmdreader/mmdadapter.cc.o
libpmdreader.a: CMakeFiles/pmdreader.dir/build.make
libpmdreader.a: CMakeFiles/pmdreader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libpmdreader.a"
	$(CMAKE_COMMAND) -P CMakeFiles/pmdreader.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pmdreader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pmdreader.dir/build: libpmdreader.a

.PHONY : CMakeFiles/pmdreader.dir/build

CMakeFiles/pmdreader.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pmdreader.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pmdreader.dir/clean

CMakeFiles/pmdreader.dir/depend:
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/remusw/Documents/Graphics/Proj4/skinning/skinning /home/remusw/Documents/Graphics/Proj4/skinning/skinning /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles/pmdreader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pmdreader.dir/depend

