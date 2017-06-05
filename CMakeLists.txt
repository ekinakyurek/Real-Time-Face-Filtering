#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#


cmake_minimum_required(VERSION 2.8.4)

PROJECT(examples)


include(./dlib/cmake)

# Tell CMake to compile a program.  We do this with the ADD_EXECUTABLE()
# statement which takes the name of the output executable and then a list of
# .cpp files to compile.  Here each example consists of only one .cpp file but
# in general you will make programs that const of many .cpp files.
#ADD_EXECUTABLE(assignment_learning_ex assignment_learning_ex.cpp)
# Then we tell it to link with dlib.
#TARGET_LINK_LIBRARIES(assignment_learning_ex dlib)




# Since there are a lot of examples I'm going to use a macro to simply this
# CMakeLists.txt file.  However, usually you will create only one executable in
# your cmake projects and use the syntax shown above.
MACRO(add_example name)
   ADD_EXECUTABLE(${name} ${name}.cpp)
   TARGET_LINK_LIBRARIES(${name} dlib )
ENDMACRO()

# if an example requires GUI, call this macro to check DLIB_NO_GUI_SUPPORT to include or exclude
MACRO(add_gui_example name)
   if (DLIB_NO_GUI_SUPPORT)
      message("No GUI support, so we won't build the ${name} example.")
   else()
      add_example(${name})
   endif()
ENDMACRO()

# The deep learning toolkit requires a compiler with essentially complete C++11
# support.  However, versions of Visual Studio prior to October 2016 didn't
# provide enough C++11 support to compile the DNN tooling, but were good enough
# to compile the rest of dlib.
if (NOT USING_OLD_VISUAL_STUDIO_COMPILER)

   if (NOT MSVC)
      # Don't try to compile this program using Visual Studio since it causes the
      # compiler to run out of RAM and to crash.  Maybe someday Visual Studio
      # won't be broken :(
      #add_example(dnn_imagenet_train_ex)
   endif()
endif()


#here we apply our macros
if (DLIB_NO_GUI_SUPPORT)
   message("No GUI support, so we won't build the webcam_face_pose_ex example.")
else()
   find_package(OpenCV QUIET)
   if (OpenCV_FOUND)
      include_directories(${OpenCV_INCLUDE_DIRS})

      ADD_EXECUTABLE(project project.cpp)
      TARGET_LINK_LIBRARIES(project dlib ${OpenCV_LIBS})

   else()
      message("OpenCV not found, so we won't build the webcam_face_pose_ex example.")
   endif()
endif()


if (DLIB_LINK_WITH_SQLITE3)
   #add_example(sqlite_ex)
endif()
