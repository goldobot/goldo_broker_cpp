#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <string>

#include "../libs/goldo_codecs/src/zmq_codecs.hpp"

#include "broker.hpp"

using namespace std;
using namespace google::protobuf;

BrokerInterface::BrokerInterface(const char *_name, int _sub_port, int _pub_port, ZmqContextId _zmq_ctxt, bool _is_connect_zmq, ZmqCodec* _codec)
{
  m_name.assign(_name);
  m_sub_socket_port = _sub_port;
  m_pub_socket_port = _pub_port;
  m_sub_socket = nullptr;
  m_pub_socket = nullptr;
  m_zmq_ctxt = _zmq_ctxt;
  m_socket_is_connect_zmq = _is_connect_zmq;
  m_codec = _codec;
}

int BrokerInterface::create_zmq_sockets()
{
  int rc=0;
  char char_buff[64];

  /* pub socket */
  if (m_pub_socket_port!=0)
  {
    m_pub_socket = zmq_socket(m_zmq_ctxt, ZMQ_PUB);
    if (m_pub_socket<0) {
      printf ("pub_socket : cannot create ZMQ_PUB socket\n");
      m_sub_socket = nullptr;
      m_pub_socket = nullptr;
      return -1;
    }

    if (m_socket_is_connect_zmq)
    {
      sprintf(char_buff, "tcp://127.0.0.1:%d", m_pub_socket_port);
      rc = zmq_connect(m_pub_socket, char_buff);
      if (rc<0) {
        printf ("pub_socket : zmq_connect() error\n");
        m_sub_socket = nullptr;
        m_pub_socket = nullptr;
        return -1;
      }
    }
    else
    {
      sprintf(char_buff, "tcp://*:%d", m_pub_socket_port);
      rc = zmq_bind(m_pub_socket, char_buff);
      if (rc<0) {
        printf ("pub_socket : cannot bind ZMQ_PUB socket\n");
        m_sub_socket = nullptr;
        m_pub_socket = nullptr;
        return -1;
      }
    }
  }

  /* sub socket */
  if (m_sub_socket_port!=0)
  {
    m_sub_socket = zmq_socket(m_zmq_ctxt, ZMQ_SUB);
    if (m_sub_socket<0) {
      printf ("sub_socket : cannot create ZMQ_SUB socket\n");
      m_sub_socket = nullptr;
      m_pub_socket = nullptr;
      return -1;
    }

    if (m_socket_is_connect_zmq)
    {
      sprintf(char_buff, "tcp://127.0.0.1:%d", m_sub_socket_port);
      rc = zmq_connect(m_sub_socket, char_buff);
      if (rc<0) {
        printf ("sub_socket : zmq_connect() error\n");
        m_sub_socket = nullptr;
        m_pub_socket = nullptr;
        return -1;
      }
    }
    else
    {
      sprintf(char_buff, "tcp://*:%d", m_sub_socket_port);
      rc = zmq_bind(m_sub_socket, char_buff);
      if (rc<0) {
        printf ("sub_socket : cannot bind ZMQ_SUB socket\n");
        m_sub_socket = nullptr;
        m_pub_socket = nullptr;
        return -1;
      }
    }

    zmq_setsockopt(m_sub_socket, ZMQ_SUBSCRIBE, "", 0);
  }

  return 0;
}

std::tuple<const std::string&, const std::string&, const std::string&> BrokerInterface::receive()
{
  unsigned char buff[1024];
  size_t bytes_read = 0;
  size_t bytes_read_total = 0;
  int64_t more=1;
  size_t more_size = sizeof(more);

  if (m_sub_socket == nullptr)
  {
    return {DontSendStr, DontSendStr, DontSendStr};
  }

  std::string msg_part1_s;
  std::string msg_part2_s;
  std::string msg_part3_s;

  msg_part1_s.clear();
  msg_part2_s.clear();
  msg_part3_s.clear();
  bytes_read_total = 0;

  {
    bytes_read = zmq_recv(m_sub_socket, buff + bytes_read_total, sizeof(buff) - bytes_read_total, 0);
    zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
    msg_part1_s.append((char *)&buff[bytes_read_total],bytes_read);
    bytes_read_total += bytes_read;
  }

  if (more)
  {
    bytes_read = zmq_recv(m_sub_socket, buff + bytes_read_total, sizeof(buff) - bytes_read_total, 0);
    zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
    msg_part2_s.append((char *)&buff[bytes_read_total],bytes_read);
    bytes_read_total += bytes_read;
  }

  if (more)
  {
    bytes_read = zmq_recv(m_sub_socket, buff + bytes_read_total, sizeof(buff) - bytes_read_total, 0);
    zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
    msg_part3_s.append((char *)&buff[bytes_read_total],bytes_read);
    bytes_read_total += bytes_read;
  }

  if (more)
  {
    printf("WARNING: received message with more than 3 parts!\n");
  }

  while(more)
  {
    bytes_read = zmq_recv(m_sub_socket, buff + bytes_read_total, sizeof(buff) - bytes_read_total, 0);
    zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
    /* POUBELLE! */
    bytes_read_total += bytes_read;
  }

#if 0 /* FIXME : DEBUG : only for nucleo ! */
  buff[bytes_read_total] = 0;
  uint16_t message_type = 0;
  memcpy (&message_type, &buff[2], sizeof(message_type));

  printf("  ZMQ DEBUG: received message_type = %d\n", message_type);
  printf("  ");
#endif

#if 0 /* FIXME : DEBUG */
  {
    int i,j,n_bytes,n_fill;

    n_bytes = (int)bytes_read_total;
    n_fill = ((bytes_read_total/16)+1)*16;

    for (i=0; i<n_bytes; i++)
    {
      printf ("%.2x ",buff[i]);
      if (i%16==15)
      {
        for (j=15; j>=0; j--)
        {
          if (buff[i-j]>=0x20)
          {
            printf ("%c",buff[i-j]);
          }
          else
          {
            printf (".");
          }
        }
        printf ("\n");
      }
    }
    for (j=n_bytes; j<n_fill; j++)
    {
      printf ("   ");
    }
    for (j=n_fill-16; j<n_bytes; j++)
    {
      if (buff[j]>=0x20)
      {
        printf ("%c",buff[j]);
      }
      else
      {
        printf (".");
      }
    }

    printf ("\n");
  }
#endif

#if 0 /* FIXME : DEBUG */
  printf ("DEBUG :   msg_part1_s = %s\n", msg_part1_s.c_str());
  printf ("DEBUG :   msg_part2_s = %s\n", msg_part2_s.c_str());
  printf ("DEBUG :   msg_part3_s = %s\n", msg_part3_s.c_str());
#endif

  auto [topic_s, msg_type_s, pb_msg_s] = m_codec->input(msg_part1_s, msg_part2_s, msg_part3_s);

  return {topic_s, msg_type_s, pb_msg_s};
}

int BrokerInterface::send(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser)
{
  int send_more = 0;

  auto [part1_s, part2_s, part3_s] = m_codec->output(_topic, _msg_type_ser, _msg_ser);

  if ((part1_s!=DontSendStr) && (part1_s!=VoidStr))
  {
    if ((part2_s!=DontSendStr) && (part2_s!=VoidStr) && (part3_s!=DontSendStr) && (part3_s!=VoidStr))
    {
      send_more = ZMQ_SNDMORE;
    }
    else
    {
      send_more = 0;
    }
    zmq_send (m_pub_socket, part1_s.data(), part1_s.size(), send_more);
  }

  if ((part2_s!=DontSendStr) && (part2_s!=VoidStr))
  {
    if ((part3_s!=DontSendStr) && (part3_s!=VoidStr))
    {
      send_more = ZMQ_SNDMORE;
    }
    else
    {
      send_more = 0;
    }
    zmq_send (m_pub_socket, part2_s.data(), part2_s.size(), send_more);
  }

  if ((part3_s!=DontSendStr) && (part3_s!=VoidStr))
  {
    zmq_send (m_pub_socket, part3_s.data(), part3_s.size(), 0);
  }

  return 0;
}


BrokerProcess::BrokerProcess()
{
  m_stop_task = false;
  m_routing = true;
  m_intfs_cnt = 0;
  m_zmq_context = zmq_ctx_new();

#if 1 /* FIXME : DEBUG */
  m_dbg_debug_intf   = nullptr;
  m_dbg_strat_intf   = nullptr;
  m_dbg_rplidar_intf = nullptr;
  m_dbg_nucleo_intf  = nullptr;
#endif
}

int BrokerProcess::create_intf(const char *_name, int _sub_port, int _pub_port, bool _is_connect_zmq, ZmqCodecType _codec_type)
{
  int rc=0;
  const char *codec_type_name = nullptr;

  ZmqCodec* new_codec = NULL;
  switch (_codec_type) {
  case ZmqCodecType::NucleoCodecType:
    new_codec = (ZmqCodec*) new NucleoCodec(_name);
    codec_type_name = "NucleoCodecType";
    break;
  case ZmqCodecType::ProtobufCodecType:
    new_codec = (ZmqCodec*) new ProtobufCodec(_name);
    codec_type_name = "ProtobufCodecType";
    break;
  case ZmqCodecType::RPLidarCodecType:
    new_codec = (ZmqCodec*) new RPLidarCodec(_name);
    codec_type_name = "RPLidarCodecType";
    break;
  default:
    printf ("BrokerProcess::create_intf() : invalid codec type\n");
    return -1;
  };

  printf ("BrokerProcess::create_intf() : creating new interface '%s' (sub_port=%d, pub_port=%d, connect_type='%s', codec_type='%s')\n",
          _name, _sub_port, _pub_port, codec_type_name, _is_connect_zmq?"CONNECT":"BIND");

  BrokerInterface* new_intf = new BrokerInterface(_name, _sub_port, _pub_port, m_zmq_context, _is_connect_zmq, new_codec);
  if (new_intf == nullptr)
  {
    printf ("BrokerProcess::create_intf() : cannot create broker interface\n");
    return -1;
  }

  rc = new_intf->create_zmq_sockets();
  if (rc != 0)
  {
    printf ("BrokerProcess::create_intf() : cannot create zmq sockets\n");
    return -1;
  }

  int i=m_intfs_cnt;
  m_intfs_cnt++;

  m_intf[i] = new_intf;
  m_poll_items[i].socket = new_intf->sub_socket();
  m_poll_items[i].fd = 0;
  m_poll_items[i].events = ZMQ_POLLIN;

  m_name_intf_map[new_intf->name()] = new_intf;

#if 1 /* FIXME : DEBUG */
  if (new_intf->name()=="debug_intf")
  {
    m_dbg_debug_intf   = new_intf;
  }
  else if (new_intf->name()=="strat_intf")
  {
    m_dbg_strat_intf   = new_intf;
  }
  else if (new_intf->name()=="rplidar_intf")
  {
    m_dbg_rplidar_intf = new_intf;
  }
  else if (new_intf->name()=="nucleo_intf")
  {
    m_dbg_nucleo_intf  = new_intf;
  }
#endif

  return 0;
}

int BrokerProcess::admin_func(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser)
{
  printf ("DEBUG : BrokerProcess::admin_func() : topic = %s\n", _topic.c_str());
  if (_topic=="broker/admin/cmd/register_callback")
  {
    printf ("DEBUG :   received REGISTER_CALLBACK\n");
    google::protobuf::StringValue pb_msg_param;
    pb_msg_param.ParseFromString(_msg_ser);
    printf ("DEBUG :   param = %s\n", pb_msg_param.value().c_str());
    if (m_dbg_strat_intf!=nullptr)
    {
      m_dbg_strat_intf->register_topic(pb_msg_param.value());
    }
  }
  else if (_topic=="broker/admin/cmd/register_forward")
  {
    printf ("DEBUG :   received REGISTER_FORWARD\n");
    google::protobuf::StringValue pb_msg_param;
    pb_msg_param.ParseFromString(_msg_ser);
    printf ("DEBUG :   param = %s\n", pb_msg_param.value().c_str());
    /* FIXME : TODO : register forwards */
  }
  else if (_topic=="broker/admin/cmd/ping")
  {
    printf ("DEBUG :   received PING\n");
    /* FIXME : TODO : PING */
  }
  else if (_topic=="broker/admin/cmd/stop")
  {
    printf ("DEBUG :   received STOP\n");
    /* FIXME : TODO : STOP */
  }
  else if (_topic=="broker/admin/cmd/start_routing")
  {
    printf ("DEBUG :   received START_ROUTING\n");
    m_routing = true;
  }
  else if (_topic=="broker/admin/cmd/stop_routing")
  {
    printf ("DEBUG :   received STOP_ROUTING\n");
    m_routing = false;
  }
  return 0;
}

int BrokerProcess::routing_func(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser)
{
  printf ("DEBUG : BrokerProcess::routing_func() : topic = %s\n", _topic.c_str());

  if (m_dbg_strat_intf!=nullptr)
  {
    if (m_dbg_strat_intf->is_registered_topic(_topic))
    {
      m_dbg_strat_intf->send(_topic, _msg_type_ser, _msg_ser);
    }

    /* FIXME : TODO : regexp topics for strat .. */
  }

  if (m_dbg_nucleo_intf!=nullptr)
  {
    if (_topic.compare(0,9,"nucleo/in")==0)
    {
      m_dbg_nucleo_intf->send(_topic, _msg_type_ser, _msg_ser);
    }
  }

  if (m_dbg_rplidar_intf!=nullptr)
  {
    if (_topic.compare(0,10,"rplidar/in")==0)
    {
      m_dbg_rplidar_intf->send(_topic, _msg_type_ser, _msg_ser);
    }
  }

  if (m_dbg_debug_intf!=nullptr)
  {
    m_dbg_debug_intf->send(_topic, _msg_type_ser, _msg_ser);
  }

  return 0;
}

void BrokerProcess::event_loop()
{
  struct timespec curr_tp;
  int curr_time_ms = 0;
  int old_time_ms = 0;

  while(!m_stop_task)
  {
    zmq_poll (m_poll_items, m_intfs_cnt, 100);

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) {
      /* FIXME : TODO : send some heartbeat message? */
      old_time_ms = curr_time_ms;
    }

#if 0 /* FIXME : DEBUG */
    for (int i=0; i<m_intfs_cnt; i++)
    {
      if(m_poll_items[i].revents && ZMQ_POLLIN)
      {            
        printf ("DEBUG : poll revent (%d)\n",i);
      }
    }
#endif

    for (int i=0; i<m_intfs_cnt; i++)
    {
      if(m_poll_items[i].revents && ZMQ_POLLIN)
      {            
        auto [topic_s, msg_type_s, pb_msg_s] = m_intf[i]->receive();
        //printf ("DEBUG : received topic = %s\n", topic_s.c_str());
        if (topic_s.compare(0,12,"broker/admin")==0)
        {
          printf ("DEBUG : received ADMIN msg\n");
          admin_func(topic_s, msg_type_s, pb_msg_s);
        }
        else if (m_routing)
        {
          printf ("DEBUG : received NORMAL msg\n");
          routing_func(topic_s, msg_type_s, pb_msg_s);
        }
      }
    }

    pthread_yield();
  }

  zmq_term(m_zmq_context);

}

