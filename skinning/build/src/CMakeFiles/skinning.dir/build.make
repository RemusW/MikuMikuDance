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
include src/CMakeFiles/skinning.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/skinning.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/skinning.dir/flags.make

src/CMakeFiles/skinning.dir/animation_loader_saver.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/animation_loader_saver.cc.o: ../src/animation_loader_saver.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/skinning.dir/animation_loader_saver.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/animation_loader_saver.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/animation_loader_saver.cc

src/CMakeFiles/skinning.dir/animation_loader_saver.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/animation_loader_saver.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/animation_loader_saver.cc > CMakeFiles/skinning.dir/animation_loader_saver.cc.i

src/CMakeFiles/skinning.dir/animation_loader_saver.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/animation_loader_saver.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/animation_loader_saver.cc -o CMakeFiles/skinning.dir/animation_loader_saver.cc.s

src/CMakeFiles/skinning.dir/bone_geometry.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/bone_geometry.cc.o: ../src/bone_geometry.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/skinning.dir/bone_geometry.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/bone_geometry.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/bone_geometry.cc

src/CMakeFiles/skinning.dir/bone_geometry.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/bone_geometry.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/bone_geometry.cc > CMakeFiles/skinning.dir/bone_geometry.cc.i

src/CMakeFiles/skinning.dir/bone_geometry.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/bone_geometry.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/bone_geometry.cc -o CMakeFiles/skinning.dir/bone_geometry.cc.s

src/CMakeFiles/skinning.dir/gui.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/gui.cc.o: ../src/gui.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/skinning.dir/gui.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/gui.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/gui.cc

src/CMakeFiles/skinning.dir/gui.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/gui.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/gui.cc > CMakeFiles/skinning.dir/gui.cc.i

src/CMakeFiles/skinning.dir/gui.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/gui.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/gui.cc -o CMakeFiles/skinning.dir/gui.cc.s

src/CMakeFiles/skinning.dir/main.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/main.cc.o: ../src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/skinning.dir/main.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/main.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/main.cc

src/CMakeFiles/skinning.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/main.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/main.cc > CMakeFiles/skinning.dir/main.cc.i

src/CMakeFiles/skinning.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/main.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/main.cc -o CMakeFiles/skinning.dir/main.cc.s

src/CMakeFiles/skinning.dir/procedure_geometry.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/procedure_geometry.cc.o: ../src/procedure_geometry.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/skinning.dir/procedure_geometry.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/procedure_geometry.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/procedure_geometry.cc

src/CMakeFiles/skinning.dir/procedure_geometry.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/procedure_geometry.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/procedure_geometry.cc > CMakeFiles/skinning.dir/procedure_geometry.cc.i

src/CMakeFiles/skinning.dir/procedure_geometry.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/procedure_geometry.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/procedure_geometry.cc -o CMakeFiles/skinning.dir/procedure_geometry.cc.s

src/CMakeFiles/skinning.dir/render_pass.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/render_pass.cc.o: ../src/render_pass.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/skinning.dir/render_pass.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/render_pass.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/render_pass.cc

src/CMakeFiles/skinning.dir/render_pass.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/render_pass.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/render_pass.cc > CMakeFiles/skinning.dir/render_pass.cc.i

src/CMakeFiles/skinning.dir/render_pass.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/render_pass.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/render_pass.cc -o CMakeFiles/skinning.dir/render_pass.cc.s

src/CMakeFiles/skinning.dir/shader_uniform.cc.o: src/CMakeFiles/skinning.dir/flags.make
src/CMakeFiles/skinning.dir/shader_uniform.cc.o: ../src/shader_uniform.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/skinning.dir/shader_uniform.cc.o"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/skinning.dir/shader_uniform.cc.o -c /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/shader_uniform.cc

src/CMakeFiles/skinning.dir/shader_uniform.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/skinning.dir/shader_uniform.cc.i"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/shader_uniform.cc > CMakeFiles/skinning.dir/shader_uniform.cc.i

src/CMakeFiles/skinning.dir/shader_uniform.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/skinning.dir/shader_uniform.cc.s"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src/shader_uniform.cc -o CMakeFiles/skinning.dir/shader_uniform.cc.s

# Object files for target skinning
skinning_OBJECTS = \
"CMakeFiles/skinning.dir/animation_loader_saver.cc.o" \
"CMakeFiles/skinning.dir/bone_geometry.cc.o" \
"CMakeFiles/skinning.dir/gui.cc.o" \
"CMakeFiles/skinning.dir/main.cc.o" \
"CMakeFiles/skinning.dir/procedure_geometry.cc.o" \
"CMakeFiles/skinning.dir/render_pass.cc.o" \
"CMakeFiles/skinning.dir/shader_uniform.cc.o"

# External object files for target skinning
skinning_EXTERNAL_OBJECTS =

bin/skinning: src/CMakeFiles/skinning.dir/animation_loader_saver.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/bone_geometry.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/gui.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/main.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/procedure_geometry.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/render_pass.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/shader_uniform.cc.o
bin/skinning: src/CMakeFiles/skinning.dir/build.make
bin/skinning: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/skinning: /usr/lib/x86_64-linux-gnu/libGL.so
bin/skinning: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/skinning: libutgraphicsutil.a
bin/skinning: /usr/lib/x86_64-linux-gnu/libjpeg.so
bin/skinning: libpmdreader.a
bin/skinning: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/skinning: src/CMakeFiles/skinning.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable ../bin/skinning"
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/skinning.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/skinning.dir/build: bin/skinning

.PHONY : src/CMakeFiles/skinning.dir/build

src/CMakeFiles/skinning.dir/clean:
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src && $(CMAKE_COMMAND) -P CMakeFiles/skinning.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/skinning.dir/clean

src/CMakeFiles/skinning.dir/depend:
	cd /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/remusw/Documents/Graphics/Proj4/skinning/skinning /home/remusw/Documents/Graphics/Proj4/skinning/skinning/src /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src /home/remusw/Documents/Graphics/Proj4/skinning/skinning/build/src/CMakeFiles/skinning.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/skinning.dir/depend

