cmake_minimum_required(VERSION 3.7)
project(goldo_broker_cpp)

add_compile_options(-std=c++17)

## load in pkg-config support
find_package(PkgConfig)

## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
  NAMES zmq
  PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
  )


add_subdirectory(libs)

add_executable(goldo_broker_cpp
  src/main.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(goldo_broker_cpp
  goldo_codecs
  goldo_pb2
  ${ZeroMQ_LIBRARY}
  protobuf
  pthread;  
)
