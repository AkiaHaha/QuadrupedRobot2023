# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaanh/Desktop/singleLeg2023

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaanh/Desktop/singleLeg2023/build

# Include any dependencies generated for this target.
include CMakeFiles/ellipseTrajectoryTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ellipseTrajectoryTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ellipseTrajectoryTest.dir/flags.make

CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o: CMakeFiles/ellipseTrajectoryTest.dir/flags.make
CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o -c /home/kaanh/Desktop/singleLeg2023/src/main.cpp

CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/singleLeg2023/src/main.cpp > CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.i

CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/singleLeg2023/src/main.cpp -o CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.s

CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o: CMakeFiles/ellipseTrajectoryTest.dir/flags.make
CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o: ../src/model.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o -c /home/kaanh/Desktop/singleLeg2023/src/model.cpp

CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/singleLeg2023/src/model.cpp > CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.i

CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/singleLeg2023/src/model.cpp -o CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.s

CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o: CMakeFiles/ellipseTrajectoryTest.dir/flags.make
CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o: ../src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o -c /home/kaanh/Desktop/singleLeg2023/src/robot.cpp

CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/singleLeg2023/src/robot.cpp > CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.i

CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/singleLeg2023/src/robot.cpp -o CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.s

CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o: CMakeFiles/ellipseTrajectoryTest.dir/flags.make
CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o: ../src/tplan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o -c /home/kaanh/Desktop/singleLeg2023/src/tplan.cpp

CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/singleLeg2023/src/tplan.cpp > CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.i

CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/singleLeg2023/src/tplan.cpp -o CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.s

CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o: CMakeFiles/ellipseTrajectoryTest.dir/flags.make
CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o: ../src/cplan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o -c /home/kaanh/Desktop/singleLeg2023/src/cplan.cpp

CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/singleLeg2023/src/cplan.cpp > CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.i

CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/singleLeg2023/src/cplan.cpp -o CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.s

# Object files for target ellipseTrajectoryTest
ellipseTrajectoryTest_OBJECTS = \
"CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o" \
"CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o" \
"CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o" \
"CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o" \
"CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o"

# External object files for target ellipseTrajectoryTest
ellipseTrajectoryTest_EXTERNAL_OBJECTS =

ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/src/main.cpp.o
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/src/model.cpp.o
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/src/robot.cpp.o
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/src/tplan.cpp.o
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/src/cplan.cpp.o
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/build.make
ellipseTrajectoryTest: /usr/aris/aris-2.3.9.230818/lib/release/libaris_lib.so
ellipseTrajectoryTest: CMakeFiles/ellipseTrajectoryTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaanh/Desktop/singleLeg2023/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable ellipseTrajectoryTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ellipseTrajectoryTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ellipseTrajectoryTest.dir/build: ellipseTrajectoryTest

.PHONY : CMakeFiles/ellipseTrajectoryTest.dir/build

CMakeFiles/ellipseTrajectoryTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ellipseTrajectoryTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ellipseTrajectoryTest.dir/clean

CMakeFiles/ellipseTrajectoryTest.dir/depend:
	cd /home/kaanh/Desktop/singleLeg2023/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaanh/Desktop/singleLeg2023 /home/kaanh/Desktop/singleLeg2023 /home/kaanh/Desktop/singleLeg2023/build /home/kaanh/Desktop/singleLeg2023/build /home/kaanh/Desktop/singleLeg2023/build/CMakeFiles/ellipseTrajectoryTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ellipseTrajectoryTest.dir/depend

