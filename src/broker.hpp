#pragma once
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <string>

#include "../libs/goldo_codecs/src/zmq_codecs.hpp"

using namespace std;

class BrokerInterface;
class BrokerProcess;

typedef void * ZmqContextId;
typedef void * ZmqSocketId;

class BrokerInterface {
public:
  BrokerInterface(const char *_name, int _sub_port, int _pub_port, ZmqContextId _zmq_ctxt, bool _is_connect_zmq, ZmqCodec* _codec);
  int create_zmq_sockets();
  std::string& name() {return m_name;};
  ZmqSocketId sub_socket() {return m_sub_socket;};
  ZmqSocketId pub_socket() {return m_pub_socket;};
  ZmqCodec*   codec()      {return m_codec;};
  std::tuple<const std::string&, const std::string&, const std::string&> receive();
  int send(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  map<std::string,int> &topics_map() {return m_topics_map;};
  void register_topic(const std::string& _topic) {m_topics_map[_topic]=1;};
  bool is_registered_topic(const std::string& _topic) {return (m_topics_map.count(_topic)>0);};
private:
  std::string m_name;

  ZmqContextId m_zmq_ctxt;
  int m_socket_is_connect_zmq;

  int m_sub_socket_port;
  ZmqSocketId m_sub_socket;

  int m_pub_socket_port;
  ZmqSocketId m_pub_socket;

  ZmqCodec* m_codec;

  map<std::string,int> m_topics_map;
};

class BrokerProcess {
public:
  BrokerProcess();
  int create_intf(const char *_name, int _sub_port, int _pub_port, bool _is_connect_zmq, ZmqCodecType _codec_type);
  int routing_func(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  int admin_func(const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  void event_loop();
  void start_routing() {m_routing = true;};
  void stop_routing() {m_routing = false;};
  void stop_broker() {m_stop_task = true;};
private:
  bool m_stop_task = false;

  bool m_routing = false;

  ZmqContextId m_zmq_context;

  static const int M_INTFS_MAX = 10;
  int m_intfs_cnt;
  BrokerInterface* m_intf[M_INTFS_MAX];
  zmq_pollitem_t m_poll_items[M_INTFS_MAX];
  map<std::string,BrokerInterface*> m_name_intf_map;

#if 1 /* FIXME : DEBUG */
  BrokerInterface* m_dbg_debug_intf;
  BrokerInterface* m_dbg_strat_intf;
  BrokerInterface* m_dbg_rplidar_intf;
  BrokerInterface* m_dbg_nucleo_intf;
#endif
};

