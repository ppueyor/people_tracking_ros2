cmake_minimum_required(VERSION 3.8)
project(people_laser_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(GSL REQUIRED)

include_directories(
  src/common
  src/linalg
  src/features
  src/PeopleDetector
)

# Common lib
add_library(common SHARED
  src/common/dynamictable.h src/common/dynamictable.hxx
  src/common/Segment.h src/common/Segment.cpp
  src/common/MultiScan.h src/common/MultiScan.cpp
)

# linalg lib
add_library(linalg SHARED
  src/linalg/macros.h
  src/linalg/Matrix.cc src/linalg/Matrix.hh
  src/linalg/Vector.cc src/linalg/Vector.hh
  src/linalg/Exception.cc src/linalg/Exception.hh
  src/linalg/Arguments.cc src/linalg/Arguments.hh
)
target_link_libraries(linalg GSL::gsl GSL::gslcblas)

# features lib
add_library(features SHARED
  src/features/Feature.h src/features/Feature.cpp
  src/features/Feature_1.h src/features/Feature_1.cpp
  src/features/Feature_2.h src/features/Feature_2.cpp
  src/features/Feature_3.h src/features/Feature_3.cpp
  src/features/Feature_6.h src/features/Feature_6.cpp
  src/features/Feature_7.h src/features/Feature_7.cpp
  src/features/Feature_8.h src/features/Feature_8.cpp
  src/features/Feature_9.h src/features/Feature_9.cpp
  src/features/Feature_10.h src/features/Feature_10.cpp
  src/features/Feature_11.h src/features/Feature_11.cpp
  src/features/Feature_12.h src/features/Feature_12.cpp
  src/features/Feature_13.h src/features/Feature_13.cpp
)
target_link_libraries(features linalg)

# PeopleDetector lib
add_library(PeopleDetector SHARED
  src/PeopleDetector/classify.cpp
  src/PeopleDetector/WeakHypothesis.h src/PeopleDetector/WeakHypothesis.cpp
  src/PeopleDetector/PeopleDetector.h src/PeopleDetector/PeopleDetector.cpp
)
target_link_libraries(PeopleDetector common linalg features GSL::gsl GSL::gslcblas)

# Training executables
add_executable(extract_features src/training/extract_features.cpp)
target_link_libraries(extract_features common linalg features GSL::gsl GSL::gslcblas)

add_executable(plot_segments src/training/plot_segments.cpp)
target_link_libraries(plot_segments common)

add_executable(segment_scans src/training/segment_scans.cpp)
target_link_libraries(segment_scans common)

add_executable(select_people src/training/select_people.cpp)
target_link_libraries(select_people common linalg features GSL::gsl GSL::gslcblas)

# Example
add_executable(template src/example/template.cpp)
target_link_libraries(template common linalg features GSL::gsl GSL::gslcblas PeopleDetector)

install(
  TARGETS common linalg features PeopleDetector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS extract_features plot_segments segment_scans select_people template
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src/common/
  DESTINATION include/common
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
)
install(
  DIRECTORY src/linalg/
  DESTINATION include/linalg
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hh"
)
install(
  DIRECTORY src/features/
  DESTINATION include/features
  FILES_MATCHING PATTERN "*.h"
)
install(
  DIRECTORY src/PeopleDetector/
  DESTINATION include/PeopleDetector
  FILES_MATCHING PATTERN "*.h"
)

install(
  EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include/common include/linalg include/features include/PeopleDetector)
ament_export_libraries(common linalg features PeopleDetector)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
