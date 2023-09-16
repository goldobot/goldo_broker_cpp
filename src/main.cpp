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

#include "../libs/goldo_codecs/src/zmq_codecs.hpp"

using namespace std;

void* m_zmq_context;
void* m_pub_socket;
void* m_sub_socket;

bool m_stop_task = false;

NucleoCodec nucleo_codec("NucleoCodec");
ProtobufCodec protobuf_codec("ProtobufCodec");

int main (int argc, char **argv)
{
  int rc;
  char char_buff[64];

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  
  printf ("TEST_PROTO\n");

  m_zmq_context = zmq_ctx_new();

  /* broker:debug:pub */
  m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_pub_socket<0) {
    printf ("pub_socket : cannot create ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", 3801);
  rc = zmq_bind(m_pub_socket, char_buff);
  if (rc<0) {
    printf ("pub_socket : cannot bind ZMQ_PUB socket\n");
    return -1;
  }

  /* broker:sucleo:sub */
  m_sub_socket = zmq_socket(m_zmq_context, ZMQ_SUB);
  if (m_sub_socket<0) {
    printf ("sub_socket : cannot create ZMQ_SUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://127.0.0.1:%d", 3001);
  rc = zmq_connect(m_sub_socket, char_buff);
  if (rc<0) {
    printf ("sub_socket : zmq_connect() error\n");
  }

  zmq_setsockopt(m_sub_socket, ZMQ_SUBSCRIBE, "", 0);

  struct timespec curr_tp;
  int curr_time_ms = 0;
  int old_time_ms = 0;

  zmq_pollitem_t poll_items[1];

  poll_items[0].socket = m_sub_socket;
  poll_items[0].fd = 0;
  poll_items[0].events = ZMQ_POLLIN;

  while(!m_stop_task)
  {
    zmq_poll (poll_items, 1, 100);

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) {
      /* FIXME : TODO : send some heartbeat message? */
      old_time_ms = curr_time_ms;
    }

    if(poll_items[0].revents && ZMQ_POLLIN)
    {            
      unsigned char buff[1024];
      size_t bytes_read = 0;
      int64_t more=1;
      size_t more_size = sizeof(more);
      while(more)
      {
        bytes_read += zmq_recv(m_sub_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);           
        zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
      }
      buff[bytes_read] = 0;
      uint16_t message_type = 0;
      memcpy (&message_type, &buff[2], sizeof(message_type));

      printf("  ZMQ DEBUG: received message_type = %d\n", message_type);
      printf("  ");
      {
        int i;
        for (i=0; i<(int)bytes_read; i++) printf ("%.2x ",buff[i]);
        printf ("\n");
      }

      std::string msg_header_s;
      std::string msg_body_s;
      std::string not_used_s;

      msg_header_s.clear();
      msg_header_s.append((char *)&buff[0],12);

      msg_body_s.clear();
      msg_body_s.append((char *)&buff[12],bytes_read-12);

      auto [topic_s, msg_type_s, pb_msg_s] = nucleo_codec.input(msg_header_s, msg_body_s, not_used_s);

      zmq_send (m_pub_socket, topic_s.data()   , topic_s.size()   , ZMQ_SNDMORE);
      zmq_send (m_pub_socket, msg_type_s.data(), msg_type_s.size(), ZMQ_SNDMORE);
      zmq_send (m_pub_socket, pb_msg_s.data()  , pb_msg_s.size()  , 0);
    }

    pthread_yield();
  }

  zmq_term(m_zmq_context);

  return 0;
}
