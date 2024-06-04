#pragma once

#include <string>
#include <cstring>
#include <map>
#include <iostream>

#include "message_types.hpp"

#include <google/protobuf/empty.pb.h>
#include <google/protobuf/wrappers.pb.h>

class NucleoInCodec;
class NucleoOutCodec;

NucleoInCodec* GetNucleoInCodec(const std::string& _topic);
NucleoOutCodec* GetNucleoOutCodec(goldobot::CommMessageType _msg_id);

void DebugDumpNucleoInCodec(void);
void DebugDumpNucleoOutCodec(void);

class NucleoInCodec {
private:
  std::string topic;
  goldobot::CommMessageType msg_id;
protected:
  std::string nucleo_in_buf;
public:
  NucleoInCodec(const char *_topic, goldobot::CommMessageType _msg_id);
  virtual google::protobuf::Message* deserialize_pb (const std::string& pb_ser) = 0;
  virtual std::string& transcode_and_serialize_nucleo_msg () = 0;
  virtual const char *get_pb_desc_name () = 0;
  virtual const std::string& get_pb_desc_string () = 0;
  virtual const char *my_name () = 0;
  const std::string &codec_topic() {return topic;}
  const goldobot::CommMessageType nucleo_msg_id() {return msg_id;}
};

class NucleoOutCodec {
private:
  std::string topic;
  goldobot::CommMessageType msg_id;
protected:
  std::string nucleo_out_buf;
public:
  NucleoOutCodec(const char *_topic, goldobot::CommMessageType _msg_id);
  virtual google::protobuf::Message* transcode (const unsigned char* data_from_nucleo, int data_size) = 0;
  virtual std::string& serialize_pb () = 0;
  virtual const char *get_pb_desc_name () = 0;
  virtual const std::string& get_pb_desc_string () = 0;
  virtual const char *my_name () = 0;
  const std::string &codec_topic() {return topic;}
  const goldobot::CommMessageType nucleo_msg_id() {return msg_id;}
};



