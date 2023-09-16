add_library(goldo_codecs STATIC
  src/zmq_codecs.cpp
  src/goldo_nucleo_transcoder.cpp
  )
  
target_include_directories(goldo_codecs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(goldo_codecs
  goldo_pb2
  )

