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
PROJECT(ros_flow VERSION 0.1.0)


#########################################
######			General config	   ######
#########################################
option(BUILD_DVS  "Add DVS msg type from ethz"  OFF)

set(CMAKE_AUTOMOC ON)

#########################################
######			Library	   ######
#########################################
file(GLOB_RECURSE ROS_FLOW_HEADERS "./include/*.h" "./include/*.inl")
file(GLOB_RECURSE ROS_FLOW_SOURCES "./src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${ROS_FLOW_HEADERS} ${ROS_FLOW_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC pthread)

#########################################
######			Dependencies       ######
#########################################

find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif()

find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${PCL_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")
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
	target_compile_definitions(${PROJECT_NAME} PUBLIC USE_ROS)
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
	target_include_directories(${PROJECT_NAME}  PUBLIC ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})
	target_include_directories(${PROJECT_NAME}  PUBLIC ${Boost_INCLUDE_DIRS} )
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

#########################################
######			Test				######
#########################################

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

#########################################
######			INSTALL 		   ######
#########################################
# include(CMakePackageConfigHelpers)


# ## INSTALLATION C++ Library
# if(NOT DEFINED CMAKE_INSTALL_PREFIX)
#   set(CMAKE_INSTALL_PREFIX "/usr/local")
# endif() 

# install(TARGETS ros_flow
#         EXPORT ros_flowTargets
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
#         )

# write_basic_package_version_file(
#   ros_flowConfigVersion.cmake
#     VERSION ${PACKAGE_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     )


# # This installs the include folder
# install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
# install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

# export(TARGETS ros_flow NAMESPACE ros_flow:: FILE ros_flow.cmake)

# # This generates ros_flowTargets.cmake
# install(EXPORT ros_flowTargets
#     FILE ros_flowTargets.cmake
#     NAMESPACE ros_flow::
#     DESTINATION lib/cmake/ros_flow
#     )

# configure_file(ros_flowConfig.cmake.in ros_flowConfig.cmake @ONLY)
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ros_flowConfig.cmake"
#         DESTINATION lib/cmake/ros_flow
#         )

# # uninstall target 
# if(NOT TARGET uninstall) 
#   configure_file( 
#       "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
#       "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
#       IMMEDIATE @ONLY) 
 
#   add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
# endif() 


 
