# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/2.8.6/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/2.8.6/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/local/Cellar/cmake/2.8.6/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/gershonbialer/IDSC-Recognizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/gershonbialer/IDSC-Recognizer

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/local/Cellar/cmake/2.8.6/bin/ccmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/local/Cellar/cmake/2.8.6/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/gershonbialer/IDSC-Recognizer/CMakeFiles /Users/gershonbialer/IDSC-Recognizer/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/gershonbialer/IDSC-Recognizer/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named idsc

# Build rule for target.
idsc: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 idsc
.PHONY : idsc

# fast build rule for target.
idsc/fast:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/build
.PHONY : idsc/fast

src/idsc.o: src/idsc.cxx.o
.PHONY : src/idsc.o

# target to build an object file
src/idsc.cxx.o:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/idsc.cxx.o
.PHONY : src/idsc.cxx.o

src/idsc.i: src/idsc.cxx.i
.PHONY : src/idsc.i

# target to preprocess a source file
src/idsc.cxx.i:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/idsc.cxx.i
.PHONY : src/idsc.cxx.i

src/idsc.s: src/idsc.cxx.s
.PHONY : src/idsc.s

# target to generate assembly for a file
src/idsc.cxx.s:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/idsc.cxx.s
.PHONY : src/idsc.cxx.s

src/reference_point.o: src/reference_point.cxx.o
.PHONY : src/reference_point.o

# target to build an object file
src/reference_point.cxx.o:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/reference_point.cxx.o
.PHONY : src/reference_point.cxx.o

src/reference_point.i: src/reference_point.cxx.i
.PHONY : src/reference_point.i

# target to preprocess a source file
src/reference_point.cxx.i:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/reference_point.cxx.i
.PHONY : src/reference_point.cxx.i

src/reference_point.s: src/reference_point.cxx.s
.PHONY : src/reference_point.s

# target to generate assembly for a file
src/reference_point.cxx.s:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/reference_point.cxx.s
.PHONY : src/reference_point.cxx.s

src/shape_descriptor.o: src/shape_descriptor.cxx.o
.PHONY : src/shape_descriptor.o

# target to build an object file
src/shape_descriptor.cxx.o:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/shape_descriptor.cxx.o
.PHONY : src/shape_descriptor.cxx.o

src/shape_descriptor.i: src/shape_descriptor.cxx.i
.PHONY : src/shape_descriptor.i

# target to preprocess a source file
src/shape_descriptor.cxx.i:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/shape_descriptor.cxx.i
.PHONY : src/shape_descriptor.cxx.i

src/shape_descriptor.s: src/shape_descriptor.cxx.s
.PHONY : src/shape_descriptor.s

# target to generate assembly for a file
src/shape_descriptor.cxx.s:
	$(MAKE) -f CMakeFiles/idsc.dir/build.make CMakeFiles/idsc.dir/src/shape_descriptor.cxx.s
.PHONY : src/shape_descriptor.cxx.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... idsc"
	@echo "... rebuild_cache"
	@echo "... src/idsc.o"
	@echo "... src/idsc.i"
	@echo "... src/idsc.s"
	@echo "... src/reference_point.o"
	@echo "... src/reference_point.i"
	@echo "... src/reference_point.s"
	@echo "... src/shape_descriptor.o"
	@echo "... src/shape_descriptor.i"
	@echo "... src/shape_descriptor.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

