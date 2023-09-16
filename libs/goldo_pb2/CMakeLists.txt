add_library(goldo_pb2 STATIC
  src/goldo/rplidar.pb.cc
  src/goldo/log.pb.cc
  src/goldo/common/geometry.pb.cc
  src/goldo/table.pb.cc
  src/goldo/strategy.pb.cc
  src/goldo/path_planner.pb.cc
  src/goldo/camera.pb.cc
  src/goldo/nucleo.pb.cc
  src/goldo/nucleo/main.pb.cc
  src/goldo/nucleo/propulsion.pb.cc
  src/goldo/nucleo/dynamixels.pb.cc
  src/goldo/nucleo/odometry.pb.cc
  src/goldo/nucleo/odrive.pb.cc
  src/goldo/nucleo/fpga.pb.cc
  src/goldo/nucleo/gpio.pb.cc
  src/goldo/nucleo/robot_simulator.pb.cc
  src/goldo/nucleo/servos.pb.cc
  src/goldo/nucleo/statistics.pb.cc
  src/goldo/nucleo/hal.pb.cc
  src/goldo/nucleo/robot.pb.cc
  src/goldo/pb2_options.pb.cc
  src/goldo/robot.pb.cc
  )
  
target_include_directories(goldo_pb2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  )
