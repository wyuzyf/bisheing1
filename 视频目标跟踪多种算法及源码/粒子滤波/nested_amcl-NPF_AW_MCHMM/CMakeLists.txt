cmake_minimum_required(VERSION 2.8.3)
project(nested_amcl)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

# The Google Glog_Pack library.
# Store the glog include directory in GLOG_INCLUDE_DIR.
# The PATHS variable may be specified to give hints for where to find logging.h
find_path(GLOG_INCLUDE_DIR
  NAMES logging.h
  PATHS /usr/local/include/glog /usr/local/include/ /usr/include/glog/ /usr/include/
)

if(GLOG_INCLUDE_DIR)
        message(STATUS "Found GLog include directory")
else()
        message(STATUS "ERROR: Could not find GLog include directory")
endif()

# Find libmlpack.so (or equivalent) and store it in MLPACK_LIBRARY.
# If this example script were smarter, it would also find other dependencies of mlpack and store them in
# an MLPACK_LIBRARIES variable instead.
find_library(GLOG_LIBRARY
  NAMES glog libglog
  PATHS /usr/lib64/ /usr/lib/ /usr/local/lib64/ /usr/local/
)

if(GLOG_LIBRARY)
        message(STATUS "Found GLog link libraries")
else()
        message(STATUS "ERROR: Could not find GLog link libraries")
endif()

find_package(catkin REQUIRED
            COMPONENTS
            roscpp
            tf
            dynamic_reconfigure
            nav_msgs
        )

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        tf
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include/amcl include/map include/sensors include/pf MCFHMM/include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(nested_amcl_pf
                    src/pf/pf.c
                    src/pf/pf_kdtree.c
                    src/pf/pf_pdf.c
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c)

add_library(nested_amcl_map
                    src/map/map.c
                    src/map/map_cspace.cpp
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c
                    src/map/map_feature.cpp)

add_library(nested_amcl_sensors
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_laser.cpp)

add_library(MCHMM_lib
					MCFHMM/src/Sampler.cpp
					MCFHMM/src/MCHMM.cpp
					MCFHMM/src/DETree.cpp
					MCFHMM/src/Sample.cpp
					MCFHMM/src/Observation.cpp
					MCFHMM/src/Timer.cpp)


set(MCFHMM_HDRs
	MCFHMM/include/Sampler.h
	MCFHMM/include/MCHMM.h
	MCFHMM/include/DETree.h
	MCFHMM/include/Sample.h
	MCFHMM/include/Observation.h
	MCFHMM/include/Timer.h
)

set(nested_amcl_pf_HDRs
	include/pf/eig3.h
	include/pf/pf.h
	include/pf/pf_kdtree.h
	include/pf/pf_pdf.h
	include/pf/pf_vector.h
)

set(nested_amcl_map_HDRs
	include/map/map.h
)

set(nested_amcl_sensors_HDRs
	include/sensors/amcl_laser.h
	include/sensors/amcl_odom.h
	include/sensors/amcl_sensor.h
)


target_link_libraries(nested_amcl_sensors nested_amcl_map nested_amcl_pf MCHMM_lib)

add_executable(nested_amcl
                       src/amcl_node.cpp ${MCFHMM_HDRs} ${nested_amcl_pf_HDRs} ${nested_amcl_map_HDRs} ${nested_amcl_sensors_HDRs})
add_dependencies(nested_amcl amcl_gencfg)

target_link_libraries(nested_amcl
    nested_amcl_sensors nested_amcl_map nested_amcl_pf MCHMM_lib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARY}
)

install( TARGETS
    nested_amcl nested_amcl_sensors nested_amcl_map nested_amcl_pf MCHMM_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Bags
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 41fe43af189ec71e5e48eb9ed661a655)
  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 752f711cf4f6e8d1d660675e2da096b0)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 4a58d1a7962914009d99000d06e5939c)
  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 6e3432115cccdca1247f6c807038e13d)
  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 27deb742fdcd3af44cf446f39f2688a8)
  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
    http://download.ros.org/data/amcl/rosie_localization_stage.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3347bf3835724cfa45e958c5c1846066)

  # Maps
  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
    http://download.ros.org/data/amcl/willow-full.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
    http://download.ros.org/data/amcl/willow-full-0.05.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  add_rostest(test/set_initial_pose.xml)
  add_rostest(test/basic_localization_stage.xml)
  add_rostest(test/small_loop_prf.xml)
  add_rostest(test/small_loop_crazy_driving_prg.xml)
  add_rostest(test/texas_greenroom_loop.xml)
  add_rostest(test/rosie_multilaser.xml)
  add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
endif()
