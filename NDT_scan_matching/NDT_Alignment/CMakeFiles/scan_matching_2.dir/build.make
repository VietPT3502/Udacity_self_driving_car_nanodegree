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
CMAKE_SOURCE_DIR = /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment

# Include any dependencies generated for this target.
include CMakeFiles/scan_matching_2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/scan_matching_2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scan_matching_2.dir/flags.make

CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o: CMakeFiles/scan_matching_2.dir/flags.make
CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o: sm2-main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o -c /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/sm2-main.cpp

CMakeFiles/scan_matching_2.dir/sm2-main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scan_matching_2.dir/sm2-main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/sm2-main.cpp > CMakeFiles/scan_matching_2.dir/sm2-main.cpp.i

CMakeFiles/scan_matching_2.dir/sm2-main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scan_matching_2.dir/sm2-main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/sm2-main.cpp -o CMakeFiles/scan_matching_2.dir/sm2-main.cpp.s

CMakeFiles/scan_matching_2.dir/helper.cpp.o: CMakeFiles/scan_matching_2.dir/flags.make
CMakeFiles/scan_matching_2.dir/helper.cpp.o: helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/scan_matching_2.dir/helper.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scan_matching_2.dir/helper.cpp.o -c /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/helper.cpp

CMakeFiles/scan_matching_2.dir/helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scan_matching_2.dir/helper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/helper.cpp > CMakeFiles/scan_matching_2.dir/helper.cpp.i

CMakeFiles/scan_matching_2.dir/helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scan_matching_2.dir/helper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/helper.cpp -o CMakeFiles/scan_matching_2.dir/helper.cpp.s

# Object files for target scan_matching_2
scan_matching_2_OBJECTS = \
"CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o" \
"CMakeFiles/scan_matching_2.dir/helper.cpp.o"

# External object files for target scan_matching_2
scan_matching_2_EXTERNAL_OBJECTS =

scan_matching_2: CMakeFiles/scan_matching_2.dir/sm2-main.cpp.o
scan_matching_2: CMakeFiles/scan_matching_2.dir/helper.cpp.o
scan_matching_2: CMakeFiles/scan_matching_2.dir/build.make
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_people.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libboost_system.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libboost_regex.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libqhull.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libfreetype.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libz.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libjpeg.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpng.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libtiff.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libexpat.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_features.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_search.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_io.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libpcl_common.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libfreetype.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
scan_matching_2: /usr/lib/x86_64-linux-gnu/libz.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libGLEW.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libSM.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libICE.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libX11.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libXext.so
scan_matching_2: /usr/lib/x86_64-linux-gnu/libXt.so
scan_matching_2: CMakeFiles/scan_matching_2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable scan_matching_2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scan_matching_2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scan_matching_2.dir/build: scan_matching_2

.PHONY : CMakeFiles/scan_matching_2.dir/build

CMakeFiles/scan_matching_2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scan_matching_2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scan_matching_2.dir/clean

CMakeFiles/scan_matching_2.dir/depend:
	cd /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment /home/vietpt/vietpt/vietpt/Udacity_self_driving_car_nanodegree/NDT_scan_matching/NDT_Alignment/CMakeFiles/scan_matching_2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scan_matching_2.dir/depend
