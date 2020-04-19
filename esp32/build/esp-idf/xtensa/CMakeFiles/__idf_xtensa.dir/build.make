# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hasan/esp/amalgam/esp32-freqcount-example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hasan/esp/amalgam/esp32-freqcount-example/build

# Include any dependencies generated for this target.
include esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/depend.make

# Include the progress variables for this target.
include esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj: /home/hasan/esp-idf/components/xtensa/debug_helpers.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj   -c /home/hasan/esp-idf/components/xtensa/debug_helpers.c

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_xtensa.dir/debug_helpers.c.i"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hasan/esp-idf/components/xtensa/debug_helpers.c > CMakeFiles/__idf_xtensa.dir/debug_helpers.c.i

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_xtensa.dir/debug_helpers.c.s"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hasan/esp-idf/components/xtensa/debug_helpers.c -o CMakeFiles/__idf_xtensa.dir/debug_helpers.c.s

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj: /home/hasan/esp-idf/components/xtensa/debug_helpers_asm.S
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building ASM object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj -c /home/hasan/esp-idf/components/xtensa/debug_helpers_asm.S

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj: /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa_asm.S
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building ASM object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj -c /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa_asm.S

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj: /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj   -c /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa.c

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.i"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa.c > CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.i

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.s"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hasan/esp-idf/components/xtensa/expression_with_stack_xtensa.c -o CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.s

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj: /home/hasan/esp-idf/components/xtensa/eri.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_xtensa.dir/eri.c.obj   -c /home/hasan/esp-idf/components/xtensa/eri.c

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_xtensa.dir/eri.c.i"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hasan/esp-idf/components/xtensa/eri.c > CMakeFiles/__idf_xtensa.dir/eri.c.i

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_xtensa.dir/eri.c.s"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hasan/esp-idf/components/xtensa/eri.c -o CMakeFiles/__idf_xtensa.dir/eri.c.s

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj: /home/hasan/esp-idf/components/xtensa/trax.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_xtensa.dir/trax.c.obj   -c /home/hasan/esp-idf/components/xtensa/trax.c

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_xtensa.dir/trax.c.i"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hasan/esp-idf/components/xtensa/trax.c > CMakeFiles/__idf_xtensa.dir/trax.c.i

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_xtensa.dir/trax.c.s"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hasan/esp-idf/components/xtensa/trax.c -o CMakeFiles/__idf_xtensa.dir/trax.c.s

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj


esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/flags.make
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj: /home/hasan/esp-idf/components/xtensa/esp32/trax_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj   -c /home/hasan/esp-idf/components/xtensa/esp32/trax_init.c

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.i"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hasan/esp-idf/components/xtensa/esp32/trax_init.c > CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.i

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.s"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && /home/hasan/.espressif/tools/xtensa-esp32-elf/esp-2019r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hasan/esp-idf/components/xtensa/esp32/trax_init.c -o CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.s

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.requires:

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.provides: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.requires
	$(MAKE) -f esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.provides.build
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.provides

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.provides.build: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj


# Object files for target __idf_xtensa
__idf_xtensa_OBJECTS = \
"CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj" \
"CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj" \
"CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj" \
"CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj" \
"CMakeFiles/__idf_xtensa.dir/eri.c.obj" \
"CMakeFiles/__idf_xtensa.dir/trax.c.obj" \
"CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj"

# External object files for target __idf_xtensa
__idf_xtensa_EXTERNAL_OBJECTS =

esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build.make
esp-idf/xtensa/libxtensa.a: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hasan/esp/amalgam/esp32-freqcount-example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libxtensa.a"
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && $(CMAKE_COMMAND) -P CMakeFiles/__idf_xtensa.dir/cmake_clean_target.cmake
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_xtensa.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build: esp-idf/xtensa/libxtensa.a

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/build

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers.c.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/debug_helpers_asm.S.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa_asm.S.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/expression_with_stack_xtensa.c.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/trax.c.obj.requires
esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires: esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/esp32/trax_init.c.obj.requires

.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/requires

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/clean:
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa && $(CMAKE_COMMAND) -P CMakeFiles/__idf_xtensa.dir/cmake_clean.cmake
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/clean

esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/depend:
	cd /home/hasan/esp/amalgam/esp32-freqcount-example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hasan/esp/amalgam/esp32-freqcount-example /home/hasan/esp-idf/components/xtensa /home/hasan/esp/amalgam/esp32-freqcount-example/build /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa /home/hasan/esp/amalgam/esp32-freqcount-example/build/esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/depend

