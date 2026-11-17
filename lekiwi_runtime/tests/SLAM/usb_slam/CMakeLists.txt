cmake_minimum_required(VERSION 3.16)
project(lekiwi_mono_usb LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ORB_SLAM3_ROOT "$ENV{HOME}/slam_ws/ORB_SLAM3" CACHE PATH "Existing ORB-SLAM3 source and library")
find_package(OpenCV 4.4 REQUIRED COMPONENTS core imgproc videoio)
find_package(Eigen3 3.1 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

set(ORB_LIBRARY "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
if(NOT EXISTS "${ORB_LIBRARY}")
  message(FATAL_ERROR "Build ORB_SLAM3 first: ${ORB_LIBRARY}")
endif()
file(READ "${ORB_SLAM3_ROOT}/src/System.cc" ORB_SYSTEM_SOURCE)
string(FIND "${ORB_SYSTEM_SOURCE}" "usb_slam: wait for workers before trajectory export" SHUTDOWN_FIX)
if(SHUTDOWN_FIX EQUAL -1)
  message(FATAL_ERROR "Apply orb_slam3_shutdown.patch and rebuild ORB_SLAM3 first.")
endif()
if("${ORB_SLAM3_ROOT}/src/System.cc" IS_NEWER_THAN "${ORB_LIBRARY}")
  message(FATAL_ERROR "System.cc changed: rebuild the ORB_SLAM3 target first.")
endif()

add_library(orb_slam3 SHARED IMPORTED)
set_target_properties(orb_slam3 PROPERTIES IMPORTED_LOCATION "${ORB_LIBRARY}")
add_executable(mono_usb mono_usb.cc)
target_compile_options(mono_usb PRIVATE -Wall -Wextra)
target_include_directories(mono_usb PRIVATE
  "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mono_usb PRIVATE orb_slam3 Eigen3::Eigen
  ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Threads::Threads)
get_filename_component(PANGOLIN_LIB_DIR "${Pangolin_DIR}/../.." ABSOLUTE)
set_target_properties(mono_usb PROPERTIES BUILD_RPATH
  "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${PANGOLIN_LIB_DIR}")
