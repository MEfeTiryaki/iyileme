cmake_minimum_required(VERSION 2.8.3)
project(iyileme)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

# to use eigen library
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  	include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
  DEPENDS
)

include_directories(
  include
	${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
add_library(
  ${PROJECT_NAME}
  src/QuadraticOptimization.cpp
  src/ParticleSwarmOptimization.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(iyileme_test
  src/test.cpp
)
add_dependencies(iyileme_test
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(iyileme_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(quad_test
  src/quad_test.cpp
)
add_dependencies(quad_test
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quad_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
