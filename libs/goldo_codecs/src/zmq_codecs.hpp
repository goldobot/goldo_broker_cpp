#pragma once
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <string>

#include <google/protobuf/empty.pb.h>
#include <google/protobuf/wrappers.pb.h>

using namespace std;
using namespace google::protobuf;

class ZmqCodec;

extern const std::string DontSendStr;
extern const std::string VoidStr;

enum class ZmqCodecType : uint32_t {
  InvalidCodecType   = 0,
  NucleoCodecType    = 1,
  ProtobufCodecType  = 2,
  RPLidarCodecType   = 3,
};

class ZmqCodec
{
private:
  std::string name;
public:
  ZmqCodec(const char *_name)
    {
      name.assign(_name);
    }
  virtual std::tuple<const std::string&, const std::string&, const std::string&> output (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser) = 0; /* this corresponds to "serialize()" in the Thomas code */
  virtual std::tuple<const std::string&, const std::string&, const std::string&> input  (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser) = 0; /* this corresponds to "deserialize()" in the Thomas code */
};

class NucleoCodec : public ZmqCodec
{
private:
  std::string header_buf;
public:
  NucleoCodec(const char *_name) : ZmqCodec(_name) {}
  virtual std::tuple<const std::string&, const std::string&, const std::string&> output (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  virtual std::tuple<const std::string&, const std::string&, const std::string&> input  (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
};

class ProtobufCodec : public ZmqCodec
{
private:
  std::string m_in_topic_buf;
  std::string m_in_msg_type_ser_buf;
  std::string m_in_msg_ser_buf;
  std::string m_out_topic_buf;
  std::string m_out_msg_type_ser_buf;
  std::string m_out_msg_ser_buf;
public:
  ProtobufCodec(const char *_name) : ZmqCodec(_name) {}
  virtual std::tuple<const std::string&, const std::string&, const std::string&> output (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  virtual std::tuple<const std::string&, const std::string&, const std::string&> input  (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
};

class RPLidarCodec : public ZmqCodec
{
private:
  std::string cmd_buf;
  std::string param_buf;
  std::string topic_buf;
  std::string pb_type_buf;
  std::string pb_ser_buf;
public:
  RPLidarCodec(const char *_name) : ZmqCodec(_name) {}
  virtual std::tuple<const std::string&, const std::string&, const std::string&> output (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
  virtual std::tuple<const std::string&, const std::string&, const std::string&> input  (const std::string& _topic, const std::string& _msg_type_ser, const std::string& _msg_ser);
};

