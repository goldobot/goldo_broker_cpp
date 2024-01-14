#include <cstddef>
#include <cstdint>
#include <tuple>
#include <string>

#include "message_types.hpp"

#include "goldo/rplidar.pb.h"
#include "goldo/common/geometry.pb.h"
#include "goldo/nucleo.pb.h"

#include "goldo_nucleo_transcoder.hpp"

#include "zmq_codecs.hpp"

using namespace std;
using namespace goldobot;
using namespace google::protobuf;

const std::string VoidStr;
const std::string DontSendStr("!DontSend!");

/* this corresponds to "serialize()" in the Thomas code */
std::tuple<const std::string&, const std::string&, const std::string&> NucleoCodec::output (
  const std::string& _topic,
  const std::string& _not_used, /* normally "_msg_type_ser" */
  const std::string& _pb_msg_ser)
{
  NucleoInCodec* encoder = GetNucleoInCodec(_topic);
  if (encoder==NULL)
  {
    cout << "NucleoCodec::output() : Error : cannot get codec for '" <<  _topic << "'" << endl;
    return {DontSendStr, DontSendStr, DontSendStr};
  }
  goldobot::CommMessageType msg_type = encoder->nucleo_msg_id();

  /* Build nucleo message header : _msg_type_struct = struct.Struct('<BBHIi') */

  header_buf.clear();

  uint8_t chan_id = 0;
  header_buf.append((const char *)&chan_id, sizeof(chan_id));

  uint8_t res1 = 0;
  header_buf.append((const char *)&res1, sizeof(res1));

  uint16_t msg_type_val = (uint16_t) msg_type;
  header_buf.append((const char *)&msg_type_val, sizeof(msg_type_val));

  uint32_t tstamp1 = 0;
  header_buf.append((const char *)&tstamp1, sizeof(tstamp1));

  int32_t tstamp2 = 0;
  header_buf.append((const char *)&tstamp2, sizeof(tstamp2));

  /* Transcode protobuf => nucleo message payload */

  encoder->deserialize_pb (_pb_msg_ser);

  const std::string& nucleo_msg_ser = encoder->transcode_and_serialize_nucleo_msg ();

  return {header_buf, nucleo_msg_ser, DontSendStr};
}

/* this corresponds to "deserialize()" in the Thomas code */
std::tuple<const std::string&, const std::string&, const std::string&> NucleoCodec::input (
  const std::string& _msg_header,
  const std::string& _msg_body,
  const std::string& _not_used)
{
  /* Parse nucleo message header : _msg_type_struct = struct.Struct('<BBHIi') */

  const unsigned char* data_from_nucleo = (const unsigned char*) _msg_header.data();
  int data_size = _msg_header.size();

  const unsigned char *ptr = data_from_nucleo;
  size_t remain_sz = data_size;

  const uint8_t comm_id = *((const uint8_t*) ptr);
  ptr += 1;
  remain_sz -= 1;
  /* FIXME : TODO : log comm_id ? */

  /* reserved (1byte) */
  ptr += 1;
  remain_sz -= 1;

  const uint16_t msg_type = *((const uint16_t*) ptr);
  ptr += sizeof(uint16_t);
  remain_sz -= sizeof(uint16_t);

  const uint32_t t_seconds = *((const uint32_t*) ptr);
  ptr += sizeof(uint32_t);
  remain_sz -= sizeof(uint32_t);
  /* FIXME : TODO : log t_seconds ? */

  const int32_t t_nanoseconds = *((const int32_t*) ptr);
  ptr += sizeof(int32_t);
  remain_sz -= sizeof(int32_t);
  /* FIXME : TODO : log t_nanoseconds ? */

  NucleoOutCodec* encoder = GetNucleoOutCodec((goldobot::CommMessageType)msg_type);
  if (encoder==NULL)
  {
    /* FIXME : DEBUG : TODO : bloody msg_type=122 (PropulsionPose)! */
    if (msg_type!=122) cout << "NucleoCodec::input() : Error : cannot get codec for " <<  msg_type << "" << endl;
    return {DontSendStr, DontSendStr, DontSendStr};
  }
  const std::string &topic = encoder->codec_topic();

  data_from_nucleo = (const unsigned char *) _msg_body.data();
  data_size = _msg_body.size();

  google::protobuf::Message* pb_msg = encoder->transcode (data_from_nucleo, data_size);

  const std::string& pb_msg_ser = encoder->serialize_pb ();

  const std::string &pb_descriptor = encoder->get_pb_desc_string();

  return {topic, pb_descriptor, pb_msg_ser};
}


std::tuple<const std::string&, const std::string&, const std::string&> ProtobufCodec::output (
  const std::string& _topic,
  const std::string& _msg_type_ser,
  const std::string& _pb_msg_ser)
{
  m_out_topic_buf        = _topic;
  m_out_msg_type_ser_buf = _msg_type_ser;
  m_out_msg_ser_buf      = _pb_msg_ser;
  return {m_out_topic_buf, m_out_msg_type_ser_buf, m_out_msg_ser_buf};
}

std::tuple<const std::string&, const std::string&, const std::string&> ProtobufCodec::input (
  const std::string& _topic,
  const std::string& _msg_type_ser,
  const std::string& _pb_msg_ser)
{
  m_in_topic_buf        = _topic;
  m_in_msg_type_ser_buf = _msg_type_ser;
  m_in_msg_ser_buf      = _pb_msg_ser;
  return {m_in_topic_buf, m_in_msg_type_ser_buf, m_in_msg_ser_buf};
}


std::tuple<const std::string&, const std::string&, const std::string&> RPLidarCodec::output (
  const std::string& _topic,
  const std::string& _not_used, /* normally "_msg_type_ser" */
  const std::string& _pb_msg_ser)
{
  uint8_t cmd_byte;

  cmd_buf.clear();
  param_buf.clear();

  if (_topic=="rplidar/in/start")
  {
    cmd_byte = 1;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    return {cmd_buf, VoidStr, DontSendStr};
  }
  else if (_topic=="rplidar/in/stop")
  {
    cmd_byte = 2;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    return {cmd_buf, VoidStr, DontSendStr};
  }
  else if (_topic=="rplidar/in/config/theta_offset")
  {
    cmd_byte = 3;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    google::protobuf::FloatValue pb_float;
    pb_float.ParseFromString(_pb_msg_ser);
    float val = pb_float.value();
    param_buf.append((const char *)&val, sizeof(val));
    return {cmd_buf, param_buf, DontSendStr};
  }
  else if (_topic=="rplidar/in/config/distance_tresholds")
  {
    float val;
    cmd_byte = 5;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    goldo::rplidar::Tresholds pb_thresh;
    pb_thresh.ParseFromString(_pb_msg_ser);
    val = pb_thresh.near();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_thresh.mid();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_thresh.far();
    param_buf.append((const char *)&val, sizeof(val));
    return {cmd_buf, param_buf, DontSendStr};
  }
  else if (_topic=="rplidar/in/robot_pose")
  {
    float val;
    cmd_byte = 4;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    goldo::common::geometry::Pose pb_pose;
    pb_pose.ParseFromString(_pb_msg_ser);
    val = pb_pose.position().x();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_pose.position().y();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_pose.yaw();
    param_buf.append((const char *)&val, sizeof(val));
    return {cmd_buf, param_buf, DontSendStr};
  }
  else if (_topic=="rplidar/in/config/autotest_enable")
  {
    cmd_byte = 6;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    google::protobuf::BoolValue pb_bool;
    pb_bool.ParseFromString(_pb_msg_ser);
    uint8_t val = pb_bool.value();
    param_buf.append((const char *)&val, sizeof(val));
    return {cmd_buf, param_buf, DontSendStr};
  }
  else if (_topic=="rplidar/in/config/send_scan_enable")
  {
    cmd_byte = 7;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    google::protobuf::BoolValue pb_bool;
    pb_bool.ParseFromString(_pb_msg_ser);
    uint8_t val = pb_bool.value();
    param_buf.append((const char *)&val, sizeof(val));
    return {cmd_buf, param_buf, DontSendStr};
  }
  else if (_topic=="rplidar/in/robot_telemetry")
  {
    float val;
    cmd_byte = 8;
    cmd_buf.append((const char *)&cmd_byte, sizeof(cmd_byte));
    goldo::nucleo::propulsion::Telemetry pb_telemetry;
    pb_telemetry.ParseFromString(_pb_msg_ser);
    val = pb_telemetry.pose().position().x();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().position().y();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().yaw();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().speed();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().yaw_rate();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().acceleration();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.pose().angular_acceleration();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.left_encoder();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.right_encoder();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.left_pwm();
    param_buf.append((const char *)&val, sizeof(val));
    val = pb_telemetry.right_pwm();
    param_buf.append((const char *)&val, sizeof(val));
    int ival;
    ival = pb_telemetry.state();
    param_buf.append((const char *)&ival, sizeof(ival));
    ival = pb_telemetry.error();
    param_buf.append((const char *)&ival, sizeof(ival));
    return {cmd_buf, param_buf, DontSendStr};
  }

  cout << "RPLidarCodec::output() : Error : bad topic : " << _topic << "" << endl;
  return {DontSendStr, DontSendStr, DontSendStr};
}


#define PB_SET_FIELD(PB_OBJ,VAR_TYPE,VAR_NAME) do {   \
  const VAR_TYPE VAR_NAME = *((const VAR_TYPE*) ptr); \
  ptr += sizeof(VAR_TYPE);                            \
  remain_sz -= sizeof(VAR_TYPE);                      \
  PB_OBJ.set_##VAR_NAME ( VAR_NAME );                 \
  } while (0);

#define PB_SET_FLOAT_FIELD(PB_OBJ,VAR_TYPE,VAR_NAME,FACTOR) do { \
  const VAR_TYPE VAR_NAME = *((const VAR_TYPE*) ptr);            \
  ptr += sizeof(VAR_TYPE);                                       \
  remain_sz -= sizeof(VAR_TYPE);                                 \
  float VAR_NAME##_f = (float)VAR_NAME*FACTOR;                   \
  PB_OBJ.set_##VAR_NAME ( VAR_NAME##_f );                        \
  } while (0);

std::tuple<const std::string&, const std::string&, const std::string&> RPLidarCodec::input (
  const std::string& _buf0,
  const std::string& _buf1,
  const std::string& _buf2)
{
  uint32_t msg_type = (uint32_t)(uint8_t)_buf0[0];

  topic_buf.clear();
  pb_type_buf.clear();
  pb_ser_buf.clear();

  switch (msg_type) {
  case 1:
  {
    topic_buf.assign("rplidar/out/scan");
    goldo::common::geometry::PointCloud point_cloud;
    const unsigned char* data_from_rplidar = (const unsigned char*) _buf2.data();
    int data_size = _buf2.size();
    point_cloud.set_num_points(data_size/8);
    point_cloud.set_data(data_from_rplidar,data_size);
    point_cloud.SerializeToString(&pb_ser_buf);
    pb_type_buf.assign(point_cloud.GetDescriptor()->full_name().c_str());
    return {topic_buf, pb_type_buf, pb_ser_buf};
  }
  case 2:
  {
    topic_buf.assign("rplidar/out/robot_detection");
    goldo::rplidar::RobotDetection robot_detection;
    const unsigned char* data_from_rplidar = (const unsigned char*) _buf1.data();
    int data_size = _buf1.size();
    const unsigned char *ptr = data_from_rplidar;
    size_t remain_sz = data_size;
    PB_SET_FIELD(robot_detection,uint32_t,timestamp_ms);
    PB_SET_FIELD(robot_detection,uint32_t,id);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,x,0.25e-3);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,y,0.25e-3);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,vx,1.0e-3);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,vy,1.0e-3);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,ax,1.0e-3);
    PB_SET_FLOAT_FIELD(robot_detection,int16_t,ay,1.0e-3);
    PB_SET_FIELD(robot_detection,uint32_t,detect_quality);
    robot_detection.SerializeToString(&pb_ser_buf);
    pb_type_buf.assign(robot_detection.GetDescriptor()->full_name().c_str());
    return {topic_buf, pb_type_buf, pb_ser_buf};
  }
  case 42:
  {
    topic_buf.assign("rplidar/out/detections");
    goldo::rplidar::Zones zones;
    const unsigned char* data_from_rplidar = (const unsigned char*) _buf1.data();
    int data_size = _buf1.size();
    const unsigned char *ptr = data_from_rplidar;
    size_t remain_sz = data_size;
    PB_SET_FIELD(zones,uint8_t,front_near);
    PB_SET_FIELD(zones,uint8_t,left_near);
    PB_SET_FIELD(zones,uint8_t,back_near);
    PB_SET_FIELD(zones,uint8_t,right_near);
    PB_SET_FIELD(zones,uint8_t,front_far);
    PB_SET_FIELD(zones,uint8_t,left_far);
    PB_SET_FIELD(zones,uint8_t,back_far);
    PB_SET_FIELD(zones,uint8_t,right_far);
    zones.SerializeToString(&pb_ser_buf);
    pb_type_buf.assign(zones.GetDescriptor()->full_name().c_str());
    return {topic_buf, pb_type_buf, pb_ser_buf};
  }
  default:
    break;
  }

  cout << "RPLidarCodec::output() : Error : bad msg_type : " << msg_type << "" << endl;
  return {DontSendStr, DontSendStr, DontSendStr};
}

