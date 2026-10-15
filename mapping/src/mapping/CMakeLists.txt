cmake_minimum_required(VERSION 3.20)

project(drone_rtabmap_mapper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(MPI REQUIRED COMPONENTS C CXX)
find_package(RTABMap REQUIRED COMPONENTS core)

find_path(CPPZMQ_INCLUDE_DIR NAMES zmq.hpp REQUIRED)
if(NOT TARGET CPPZMQ::CPPZMQ)
  add_library(CPPZMQ::CPPZMQ INTERFACE IMPORTED)
  set_target_properties(CPPZMQ::CPPZMQ PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CPPZMQ_INCLUDE_DIR}"
  )
endif()

find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(rtabmap_mapper rtabmap_mapper.cpp)
target_compile_options(rtabmap_mapper PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rtabmap_mapper
  PRIVATE
    rtabmap::core
    ${OpenCV_LIBS}
    gz-msgs10::core
    gz-transport13::core
)

add_executable(rtabmap_db_check rtabmap_db_check.cpp)
target_compile_options(rtabmap_db_check PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rtabmap_db_check PRIVATE rtabmap::core)

install(TARGETS rtabmap_mapper rtabmap_db_check RUNTIME DESTINATION bin)
