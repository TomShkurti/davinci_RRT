cmake_minimum_required(VERSION 2.8.3)
project(algrob_project)

#This is needed as part of the migration to ros jade and later
find_package(Eigen3)

# this is depreciated but still necessary for now.
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
  set(EIGEN_DEP Eigen)
  # message("Found Eigen ${EIGEN_INCLUDE_DIR}")
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(EIGEN_DEP Eigen3)
  # message("Found Eigen3 ${EIGEN3_INCLUDE_DIR}")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
	${catkin_INCLUDE_DIRS}
	include
	${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
	DEPENDS roscpp ${EGIEN_DEP}
	LIBRARIES
	CATKIN_DEPENDS roscpp
)

add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executables
add_executable(test_pather src/test_plan_path.cpp)
add_library(rert src/rert.cpp)
add_library(davinci_model src/davinci_model.cpp)
add_library(tri_intersect src/tri_intersect.cpp)

#Link Executables
target_link_libraries(test_pather rert davinci_model tri_intersect ${catkin_LIBRARIES})
#target_link_libraries(davinci_to_sim ${catkin_LIBRARIES})
