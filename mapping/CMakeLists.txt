cmake_minimum_required(VERSION 2.8.3)
project(mapping)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            tf2
			grid_map_core
			grid_map_ros
			grid_map_cv
			grid_map_filters
			grid_map_loader
			grid_map_msgs
			grid_map_octomap
			cv_bridge
			octomap_msgs
			env_model_msgs

        )

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)
find_package(octomap REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()




catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        map_server_image_loader
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        tf2
		env_model_msgs

)

include_directories(
    include
    ${BULLET_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
  	${EIGEN3_INCLUDE_DIR}
 	${OCTOMAP_INCLUDE_DIR}
)


add_library(map_reps 
	src/map_reps/gridmap.cpp
	src/map_reps/occupancy_grid.cpp
	src/map_reps/map_reps_ros.cpp
	src/map_reps/map_loader.cpp
	src/map_reps/map_converter.cpp
	src/env_model/occupancy_grid.cpp
	src/env_model/grid_map.cpp
	#src/env_model/env_model_ros.cpp
	src/env_model/env_model_loader.cpp
	#src/env_model/env_model.cpp
)


add_dependencies(map_reps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_reps
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
	${YAMLCPP_LIBRARIES}
)


add_executable(env_model src/env_model/env_model_ros.cpp)
add_dependencies(env_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(env_model
    map_reps
    ${catkin_LIBRARIES}
)


add_executable(mapping_server src/server_node.cpp)
add_dependencies(mapping_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapping_server
    map_reps
    ${catkin_LIBRARIES}
)


add_executable(mapping-map_saver src/map_saver.cpp)
add_dependencies(mapping-map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(mapping-map_saver PROPERTIES OUTPUT_NAME map_saver)
target_link_libraries(mapping-map_saver
    ${catkin_LIBRARIES}
)


# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()

## Tests
if(CATKIN_ENABLE_TESTING)
  copy_test_data( FILES
      test/testmap.bmp
      test/testmap.png )
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp test/test_constants.cpp)
  target_link_libraries(${PROJECT_NAME}_utest
    mapping_image_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
  )

  add_executable(rtest_mapping test/rtest.cpp test/test_constants.cpp)
  add_dependencies(rtest_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries( rtest_mapping
      gtest
      ${catkin_LIBRARIES}
  )

  # This has to be done after we've already built targets, or catkin variables get borked
  find_package(rostest REQUIRED)
  add_rostest(test/rtest.xml)
endif()




## Install executables and/or libraries
install(TARGETS mapping-map_saver map_reps mapping_server env_model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Install excutable python script
install(
    PROGRAMS
      scripts/crop_map
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
