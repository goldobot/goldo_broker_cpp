/* 
TO GET COMPILE FLAGS:
pkg-config --cflags --libs protobuf
TO BUILD : 
g++ -std=c++17 -I../libs/goldo_pb2/src -I../libs/goldo_codecs/src -c main.cpp 
g++ -o goldo_broker_cpp main.o ../test/goldo_codecs.a ../test/goldo_pb2_cpp.a -lpthread -lprotobuf -lzmq
*/

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>

#include "broker.hpp"

using namespace std;

BrokerProcess the_broker;

int main (int argc, char **argv)
{
  int rc;
  char char_buff[64];

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  
  printf ("TEST GOLDO_BROKER_CPP\n");

  the_broker.create_intf("nucleo_intf" , 3001, 3002, true , ZmqCodecType::NucleoCodecType);
  the_broker.create_intf("rplidar_intf", 3102, 3101, true , ZmqCodecType::RPLidarCodecType);
  the_broker.create_intf("strat_intf"  , 3702, 3701, false, ZmqCodecType::ProtobufCodecType);
  the_broker.create_intf("debug_intf"  , 3802, 3801, false, ZmqCodecType::ProtobufCodecType);

  the_broker.event_loop();

  return 0;
}
