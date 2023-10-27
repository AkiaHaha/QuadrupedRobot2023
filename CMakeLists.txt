cmake_minimum_required(VERSION 3.12)

project(ellipseTrajectoryTest)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris/aris-2.3.6.230423)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-2.3.2.220803)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


add_executable(ellipseTrajectoryTest src/main.cpp  src/model.cpp  src/robot.cpp src/robot.h src/tplan.cpp src/tplan.h  src/cplan.cpp src/cplan.h)

if(UNIX)
    target_link_libraries(ellipseTrajectoryTest ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(ellipseTrajectoryTest ${aris_LIBRARIES})
endif(WIN32)