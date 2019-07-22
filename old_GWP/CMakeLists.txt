cmake_minimum_required(VERSION 2.8)

project(GenerateWalkPattern)

set(DCMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system filesystem iostreams)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(GenerateWalkPattern main.cpp PreviewControl.cpp PreviewControl.h FootPlanner.cpp FootPlanner.h CurveFitting.cpp CurveFitting.h)

target_link_libraries(GenerateWalkPattern ${Boost_LIBRARIES})

