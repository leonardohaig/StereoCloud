cmake_minimum_required(VERSION 3.5)
set(PROJECTNAME StereoCloud)
project(${PROJECTNAME})
set(CMAKE_CXX_STANDARD 11)

MESSAGE("===========================================================")
MESSAGE("=============This projece name is " ${PROJECTNAME} "==============")
MESSAGE("===========================================================")



IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

message("===================================")
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
if( ${CMAKE_BUILD_TYPE} STREQUAL Release )
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
    message("程序将进行O3优化")
else()
    message("程序未采用O3优化")
endif()
message("===================================")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions(${PCL_DEFINITIONS})


aux_source_directory(./ SRC_DIR)

add_executable(${PROJECTNAME} ${SRC_DIR})

target_link_libraries(${PROJECTNAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} -pthread)
