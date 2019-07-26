# cmake_minimum_required(VERSION 2.8)
# 
# project(bayesianFilter)
# 
# include_directories(include)
# 
# file(GLOB SOURCES "src/*.cpp")
# 
# add_executable(outz ${SOURCES})
# 
# # unit testing
# enable_testing()
# 
# find_package(Boost REQUIRED COMPONENTS unit_test_framework)
# 
# include_directories(${Boost_INCLUDE_DIR})
# include_directories(include)
# 
# add_executable(testMotionModel test/test_motionmodel.cpp)
# 
# target_link_libraries(testMotionModel ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

cmake_minimum_required(VERSION 3.5)
project(bayesianFilter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

file(GLOB_RECURSE INC_FILES /include/*.h)
file(GLOB_RECURSE SRC_FILES /src/*.cpp)

find_package(Boost COMPONENTS filesystem system unit_test_framework REQUIRED)

# add_subdirectory(test)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

set(SOURCE_FILES src/main.cpp ${INC_FILES} ${SRC_FILES})
add_executable(outz ${SOURCE_FILES})
target_link_libraries(outz ${BOOST_LIBRARIES})