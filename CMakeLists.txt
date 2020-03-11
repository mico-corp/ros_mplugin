##---------------------------------------------------------------------------------------------------------------------
##  ROS FLOW-PLUGIN
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 - Pablo Ramon Soria (a.k.a. Bardo91) 
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
##  and associated documentation files (the "Software"), to deal in the Software without restriction, 
##  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial 
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
PROJECT(mico VERSION 0.1)
set(PLUGIN_NAME ${PROJECT_NAME}-ros_mplugin)


#########################################
######			General config	   ######
#########################################
option(BUILD_DVS  "Add DVS msg type from ethz"  OFF)

set(CMAKE_AUTOMOC ON)

#########################################
######			Library	   ######
#########################################
file(GLOB_RECURSE ROS_MPLUGIN_HEADERS "./include/*.h" "./include/*.inl")
file(GLOB_RECURSE ROS_MPLUGIN_SOURCES "./src/*.cpp")

add_library(${PLUGIN_NAME} SHARED ${ROS_MPLUGIN_HEADERS} ${ROS_MPLUGIN_SOURCES})
target_compile_features(${PLUGIN_NAME} PUBLIC cxx_std_17)
set_target_properties(${PLUGIN_NAME} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${PLUGIN_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC pthread)

#########################################
######			Dependencies       ######
#########################################

find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
  target_include_directories(${PLUGIN_NAME} PUBLIC "/usr/include/eigen3")
endif()

find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
  target_include_directories(${PLUGIN_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(${PLUGIN_NAME} LINK_PRIVATE ${PCL_LIBRARIES})
  target_compile_definitions(${PLUGIN_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")
endif()

if(${BUILD_DVS})
  include(cmake/install_ros_macros.cmake)

  if(NOT EXISTS "~/.flow/thirdparty/catkin_ethz")
    install_ros_ethz()
  endif()
endif()

find_package(Boost COMPONENTS system)
find_package(catkin COMPONENTS roscpp tf rosbag cv_bridge image_transport sensor_msgs geometry_msgs )#dvs_msgs)
if(${catkin_FOUND})
	catkin_package(
		CATKIN_DEPENDS roscpp tf rosbag cv_bridge image_transport sensor_msgs geometry_msgs #dvs_msgs
		INCLUDE_DIRS include
	)
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
	target_include_directories(${PLUGIN_NAME}  PUBLIC ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})
	target_include_directories(${PLUGIN_NAME}  PUBLIC ${Boost_INCLUDE_DIRS} )
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
	add_dependencies(${PLUGIN_NAME} ${catkin_EXPORTED_TARGETS})
	# target_compile_definitions(${PLUGIN_NAME} PUBLIC USE_ROS)
  endif()

  find_package(flow REQUIRED)
  if (${flow_FOUND})
    target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC flow::flow)
    target_compile_definitions(${PLUGIN_NAME} PUBLIC HAS_FLOW)
  endif(${flow_FOUND})

set(MICO_DEPS "core")
foreach(DEP ${MICO_DEPS})
  find_package(mico-${DEP} REQUIRED HINTS "/usr/local/lib/cmake/mico")
  target_link_libraries(${PLUGIN_NAME} LINK_PRIVATE mico::mico-${DEP})
endforeach()

#########################################
######			Test				######
#########################################

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

#########################################
######			INSTALL 		   ######
#########################################

include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 


install(TARGETS ${PLUGIN_NAME} EXPORT ${PLUGIN_NAME}-targets
		COMPONENT ${PLUGIN_NAME}
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include
	  )


install(EXPORT ${PLUGIN_NAME}-targets
	FILE "${PLUGIN_NAME}-targets.cmake"
	NAMESPACE ${PLUGIN_NAME}::
	DESTINATION lib/cmake/${PLUGIN_NAME}
	COMPONENT ${PLUGIN_NAME}
)

# configure_file("${PLUGIN_NAME}-config.cmake.in"
# 	"${CMAKE_BINARY_DIR}/${PLUGIN_NAME}-config.cmake"
# 	@ONLY
# )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${PLUGIN_NAME}-config-version.cmake"
  VERSION ${version}
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    # "${CMAKE_BINARY_DIR}/${PLUGIN_NAME}-config.cmake"
    "${CMAKE_BINARY_DIR}/${PLUGIN_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PLUGIN_NAME}
  COMPONENT ${PLUGIN_NAME}
)

# DESTINATION will be automatically prefixed by ${CMAKE_INSTALL_PREFIX}
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  COMPONENT ${PLUGIN_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 

# FLOW INSTALL target 
add_custom_target(flow_install
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/lib${PLUGIN_NAME}.so $ENV{HOME}/.flow/plugins/lib${PLUGIN_NAME}.so
)
