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
CMAKE_SOURCE_DIR = /home/cs18/Desktop/HW8

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cs18/Desktop/HW8/build

# Include any dependencies generated for this target.
include CGL/deps/glfw/src/CMakeFiles/glfw.dir/depend.make

# Include the progress variables for this target.
include CGL/deps/glfw/src/CMakeFiles/glfw.dir/progress.make

# Include the compile flags for this target's objects.
include CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o: ../CGL/deps/glfw/src/context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/context.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/context.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/context.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/context.c > CMakeFiles/glfw.dir/context.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/context.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/context.c -o CMakeFiles/glfw.dir/context.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o: ../CGL/deps/glfw/src/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/init.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/init.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/init.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/init.c > CMakeFiles/glfw.dir/init.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/init.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/init.c -o CMakeFiles/glfw.dir/init.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o: ../CGL/deps/glfw/src/input.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/input.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/input.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/input.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/input.c > CMakeFiles/glfw.dir/input.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/input.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/input.c -o CMakeFiles/glfw.dir/input.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: ../CGL/deps/glfw/src/monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/monitor.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/monitor.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/monitor.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/monitor.c > CMakeFiles/glfw.dir/monitor.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/monitor.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/monitor.c -o CMakeFiles/glfw.dir/monitor.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o: ../CGL/deps/glfw/src/window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/window.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/window.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/window.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/window.c > CMakeFiles/glfw.dir/window.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/window.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/window.c -o CMakeFiles/glfw.dir/window.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o: ../CGL/deps/glfw/src/x11_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_init.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_init.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_init.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_init.c > CMakeFiles/glfw.dir/x11_init.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_init.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_init.c -o CMakeFiles/glfw.dir/x11_init.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o: ../CGL/deps/glfw/src/x11_monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_monitor.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_monitor.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_monitor.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_monitor.c > CMakeFiles/glfw.dir/x11_monitor.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_monitor.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_monitor.c -o CMakeFiles/glfw.dir/x11_monitor.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o: ../CGL/deps/glfw/src/x11_window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_window.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_window.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_window.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_window.c > CMakeFiles/glfw.dir/x11_window.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_window.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/x11_window.c -o CMakeFiles/glfw.dir/x11_window.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o: ../CGL/deps/glfw/src/xkb_unicode.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/xkb_unicode.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/xkb_unicode.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/xkb_unicode.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/xkb_unicode.c > CMakeFiles/glfw.dir/xkb_unicode.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/xkb_unicode.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/xkb_unicode.c -o CMakeFiles/glfw.dir/xkb_unicode.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o: ../CGL/deps/glfw/src/linux_joystick.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/linux_joystick.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/linux_joystick.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/linux_joystick.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/linux_joystick.c > CMakeFiles/glfw.dir/linux_joystick.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/linux_joystick.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/linux_joystick.c -o CMakeFiles/glfw.dir/linux_joystick.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o: ../CGL/deps/glfw/src/posix_time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/posix_time.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_time.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/posix_time.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_time.c > CMakeFiles/glfw.dir/posix_time.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/posix_time.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_time.c -o CMakeFiles/glfw.dir/posix_time.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: ../CGL/deps/glfw/src/posix_tls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/posix_tls.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_tls.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/posix_tls.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_tls.c > CMakeFiles/glfw.dir/posix_tls.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/posix_tls.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/posix_tls.c -o CMakeFiles/glfw.dir/posix_tls.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o


CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o: CGL/deps/glfw/src/CMakeFiles/glfw.dir/flags.make
CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o: ../CGL/deps/glfw/src/glx_context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/glx_context.c.o   -c /home/cs18/Desktop/HW8/CGL/deps/glfw/src/glx_context.c

CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/glx_context.c.i"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cs18/Desktop/HW8/CGL/deps/glfw/src/glx_context.c > CMakeFiles/glfw.dir/glx_context.c.i

CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/glx_context.c.s"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cs18/Desktop/HW8/CGL/deps/glfw/src/glx_context.c -o CMakeFiles/glfw.dir/glx_context.c.s

CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires:

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides: CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires
	$(MAKE) -f CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides.build
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides

CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides.build: CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o


# Object files for target glfw
glfw_OBJECTS = \
"CMakeFiles/glfw.dir/context.c.o" \
"CMakeFiles/glfw.dir/init.c.o" \
"CMakeFiles/glfw.dir/input.c.o" \
"CMakeFiles/glfw.dir/monitor.c.o" \
"CMakeFiles/glfw.dir/window.c.o" \
"CMakeFiles/glfw.dir/x11_init.c.o" \
"CMakeFiles/glfw.dir/x11_monitor.c.o" \
"CMakeFiles/glfw.dir/x11_window.c.o" \
"CMakeFiles/glfw.dir/xkb_unicode.c.o" \
"CMakeFiles/glfw.dir/linux_joystick.c.o" \
"CMakeFiles/glfw.dir/posix_time.c.o" \
"CMakeFiles/glfw.dir/posix_tls.c.o" \
"CMakeFiles/glfw.dir/glx_context.c.o"

# External object files for target glfw
glfw_EXTERNAL_OBJECTS =

CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/build.make
CGL/deps/glfw/src/libglfw3.a: CGL/deps/glfw/src/CMakeFiles/glfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cs18/Desktop/HW8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking C static library libglfw3.a"
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean_target.cmake
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CGL/deps/glfw/src/CMakeFiles/glfw.dir/build: CGL/deps/glfw/src/libglfw3.a

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/build

CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires
CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires: CGL/deps/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires

.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/requires

CGL/deps/glfw/src/CMakeFiles/glfw.dir/clean:
	cd /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean.cmake
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/clean

CGL/deps/glfw/src/CMakeFiles/glfw.dir/depend:
	cd /home/cs18/Desktop/HW8/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs18/Desktop/HW8 /home/cs18/Desktop/HW8/CGL/deps/glfw/src /home/cs18/Desktop/HW8/build /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src /home/cs18/Desktop/HW8/build/CGL/deps/glfw/src/CMakeFiles/glfw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CGL/deps/glfw/src/CMakeFiles/glfw.dir/depend
