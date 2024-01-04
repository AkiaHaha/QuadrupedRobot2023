cmake_minimum_required(VERSION 3.12)

project(QuadrupedRobot)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


file(GLOB_RECURSE SOURCES "src/*.cpp")

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

add_executable(${PROJECT_NAME} ${SOURCES} )

if(UNIX)
    target_link_libraries(QuadrupedRobot ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(QuadrupedRobot ${aris_LIBRARIES})
endif(WIN32)