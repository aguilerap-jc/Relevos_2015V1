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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aguilerapjc/JC/NAO/naoWS/relevos

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64

# Include any dependencies generated for this target.
include CMakeFiles/relevos.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/relevos.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/relevos.dir/flags.make

CMakeFiles/relevos.dir/main.cpp.o: CMakeFiles/relevos.dir/flags.make
CMakeFiles/relevos.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/relevos.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/relevos.dir/main.cpp.o -c /home/aguilerapjc/JC/NAO/naoWS/relevos/main.cpp

CMakeFiles/relevos.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/relevos.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/aguilerapjc/JC/NAO/naoWS/relevos/main.cpp > CMakeFiles/relevos.dir/main.cpp.i

CMakeFiles/relevos.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/relevos.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/aguilerapjc/JC/NAO/naoWS/relevos/main.cpp -o CMakeFiles/relevos.dir/main.cpp.s

CMakeFiles/relevos.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/relevos.dir/main.cpp.o.requires

CMakeFiles/relevos.dir/main.cpp.o.provides: CMakeFiles/relevos.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/relevos.dir/build.make CMakeFiles/relevos.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/relevos.dir/main.cpp.o.provides

CMakeFiles/relevos.dir/main.cpp.o.provides.build: CMakeFiles/relevos.dir/main.cpp.o

CMakeFiles/relevos.dir/NaoVision.cpp.o: CMakeFiles/relevos.dir/flags.make
CMakeFiles/relevos.dir/NaoVision.cpp.o: ../NaoVision.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/relevos.dir/NaoVision.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/relevos.dir/NaoVision.cpp.o -c /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoVision.cpp

CMakeFiles/relevos.dir/NaoVision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/relevos.dir/NaoVision.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoVision.cpp > CMakeFiles/relevos.dir/NaoVision.cpp.i

CMakeFiles/relevos.dir/NaoVision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/relevos.dir/NaoVision.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoVision.cpp -o CMakeFiles/relevos.dir/NaoVision.cpp.s

CMakeFiles/relevos.dir/NaoVision.cpp.o.requires:
.PHONY : CMakeFiles/relevos.dir/NaoVision.cpp.o.requires

CMakeFiles/relevos.dir/NaoVision.cpp.o.provides: CMakeFiles/relevos.dir/NaoVision.cpp.o.requires
	$(MAKE) -f CMakeFiles/relevos.dir/build.make CMakeFiles/relevos.dir/NaoVision.cpp.o.provides.build
.PHONY : CMakeFiles/relevos.dir/NaoVision.cpp.o.provides

CMakeFiles/relevos.dir/NaoVision.cpp.o.provides.build: CMakeFiles/relevos.dir/NaoVision.cpp.o

CMakeFiles/relevos.dir/NaoMovement.cpp.o: CMakeFiles/relevos.dir/flags.make
CMakeFiles/relevos.dir/NaoMovement.cpp.o: ../NaoMovement.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/relevos.dir/NaoMovement.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/relevos.dir/NaoMovement.cpp.o -c /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoMovement.cpp

CMakeFiles/relevos.dir/NaoMovement.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/relevos.dir/NaoMovement.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoMovement.cpp > CMakeFiles/relevos.dir/NaoMovement.cpp.i

CMakeFiles/relevos.dir/NaoMovement.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/relevos.dir/NaoMovement.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/aguilerapjc/JC/NAO/naoWS/relevos/NaoMovement.cpp -o CMakeFiles/relevos.dir/NaoMovement.cpp.s

CMakeFiles/relevos.dir/NaoMovement.cpp.o.requires:
.PHONY : CMakeFiles/relevos.dir/NaoMovement.cpp.o.requires

CMakeFiles/relevos.dir/NaoMovement.cpp.o.provides: CMakeFiles/relevos.dir/NaoMovement.cpp.o.requires
	$(MAKE) -f CMakeFiles/relevos.dir/build.make CMakeFiles/relevos.dir/NaoMovement.cpp.o.provides.build
.PHONY : CMakeFiles/relevos.dir/NaoMovement.cpp.o.provides

CMakeFiles/relevos.dir/NaoMovement.cpp.o.provides.build: CMakeFiles/relevos.dir/NaoMovement.cpp.o

# Object files for target relevos
relevos_OBJECTS = \
"CMakeFiles/relevos.dir/main.cpp.o" \
"CMakeFiles/relevos.dir/NaoVision.cpp.o" \
"CMakeFiles/relevos.dir/NaoMovement.cpp.o"

# External object files for target relevos
relevos_EXTERNAL_OBJECTS =

sdk/bin/relevos: CMakeFiles/relevos.dir/main.cpp.o
sdk/bin/relevos: CMakeFiles/relevos.dir/NaoVision.cpp.o
sdk/bin/relevos: CMakeFiles/relevos.dir/NaoMovement.cpp.o
sdk/bin/relevos: CMakeFiles/relevos.dir/build.make
sdk/bin/relevos: /usr/local/lib/libopencv_highgui.so
sdk/bin/relevos: /usr/local/lib/libopencv_video.so
sdk/bin/relevos: /usr/local/lib/libopencv_imgproc.so
sdk/bin/relevos: /usr/local/lib/libopencv_core.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalcommon.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalsoap.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/librttools.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalthread.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_signals-mt.a
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_program_options-mt.a
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalvalue.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libtinyxml.so
sdk/bin/relevos: /usr/lib/x86_64-linux-gnu/librt.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libqi.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_filesystem-mt.a
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_thread-mt.a
sdk/bin/relevos: /usr/lib/x86_64-linux-gnu/libpthread.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_system-mt.a
sdk/bin/relevos: /usr/lib/x86_64-linux-gnu/libdl.so
sdk/bin/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalerror.so
sdk/bin/relevos: CMakeFiles/relevos.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable sdk/bin/relevos"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/relevos.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/relevos.dir/build: sdk/bin/relevos
.PHONY : CMakeFiles/relevos.dir/build

# Object files for target relevos
relevos_OBJECTS = \
"CMakeFiles/relevos.dir/main.cpp.o" \
"CMakeFiles/relevos.dir/NaoVision.cpp.o" \
"CMakeFiles/relevos.dir/NaoMovement.cpp.o"

# External object files for target relevos
relevos_EXTERNAL_OBJECTS =

CMakeFiles/CMakeRelink.dir/relevos: CMakeFiles/relevos.dir/main.cpp.o
CMakeFiles/CMakeRelink.dir/relevos: CMakeFiles/relevos.dir/NaoVision.cpp.o
CMakeFiles/CMakeRelink.dir/relevos: CMakeFiles/relevos.dir/NaoMovement.cpp.o
CMakeFiles/CMakeRelink.dir/relevos: CMakeFiles/relevos.dir/build.make
CMakeFiles/CMakeRelink.dir/relevos: /usr/local/lib/libopencv_highgui.so
CMakeFiles/CMakeRelink.dir/relevos: /usr/local/lib/libopencv_video.so
CMakeFiles/CMakeRelink.dir/relevos: /usr/local/lib/libopencv_imgproc.so
CMakeFiles/CMakeRelink.dir/relevos: /usr/local/lib/libopencv_core.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalcommon.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalsoap.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/librttools.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalthread.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_signals-mt.a
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_program_options-mt.a
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalvalue.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libtinyxml.so
CMakeFiles/CMakeRelink.dir/relevos: /usr/lib/x86_64-linux-gnu/librt.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libqi.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_filesystem-mt.a
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_thread-mt.a
CMakeFiles/CMakeRelink.dir/relevos: /usr/lib/x86_64-linux-gnu/libpthread.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_system-mt.a
CMakeFiles/CMakeRelink.dir/relevos: /usr/lib/x86_64-linux-gnu/libdl.so
CMakeFiles/CMakeRelink.dir/relevos: /home/aguilerapjc/JC/NAO/devtools/naoqi-sdk-1.14.5-linux64/lib/libalerror.so
CMakeFiles/CMakeRelink.dir/relevos: CMakeFiles/relevos.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable CMakeFiles/CMakeRelink.dir/relevos"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/relevos.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
CMakeFiles/relevos.dir/preinstall: CMakeFiles/CMakeRelink.dir/relevos
.PHONY : CMakeFiles/relevos.dir/preinstall

CMakeFiles/relevos.dir/requires: CMakeFiles/relevos.dir/main.cpp.o.requires
CMakeFiles/relevos.dir/requires: CMakeFiles/relevos.dir/NaoVision.cpp.o.requires
CMakeFiles/relevos.dir/requires: CMakeFiles/relevos.dir/NaoMovement.cpp.o.requires
.PHONY : CMakeFiles/relevos.dir/requires

CMakeFiles/relevos.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/relevos.dir/cmake_clean.cmake
.PHONY : CMakeFiles/relevos.dir/clean

CMakeFiles/relevos.dir/depend:
	cd /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aguilerapjc/JC/NAO/naoWS/relevos /home/aguilerapjc/JC/NAO/naoWS/relevos /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64 /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64 /home/aguilerapjc/JC/NAO/naoWS/relevos/build-naoqi64/CMakeFiles/relevos.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/relevos.dir/depend

