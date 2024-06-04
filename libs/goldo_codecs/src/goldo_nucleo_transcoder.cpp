/*
Create pb2 static lib with:
ar crf goldo_pb2_cpp.a ../obj/*.o
*/

#include <fcntl.h> 
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <string>
#include <cstring>
#include <map>
#include <iostream>

#include "message_types.hpp"

#include <google/protobuf/empty.pb.h>
#include <google/protobuf/wrappers.pb.h>
#include "goldo/nucleo.pb.h"
#include "goldo/nucleo/main.pb.h"
#include "goldo/nucleo/gpio.pb.h"
#include "goldo/nucleo/odrive.pb.h"
#include "goldo/nucleo/servos.pb.h"
#include "goldo/nucleo/dynamixels.pb.h"
#include "goldo/nucleo/fpga.pb.h"
#include "goldo/nucleo/odometry.pb.h"
#include "goldo/nucleo/robot.pb.h"

#include "goldo_nucleo_transcoder.hpp"

using namespace std;
using namespace goldobot;

map<std::string,NucleoInCodec*> NucleoInCodecMap;
map<goldobot::CommMessageType,NucleoOutCodec*> NucleoOutCodecMap;

NucleoInCodec::NucleoInCodec(const char *_topic, goldobot::CommMessageType _msg_id)
{
  topic.assign(_topic);
  msg_id = _msg_id;
  NucleoInCodecMap[topic] = this;
}

NucleoOutCodec::NucleoOutCodec(const char *_topic, goldobot::CommMessageType _msg_id)
{
  topic.assign(_topic);
  msg_id = _msg_id;
  NucleoOutCodecMap[msg_id] = this;
}

NucleoInCodec* GetNucleoInCodec(const std::string& _topic)
{
  if (NucleoInCodecMap.count(_topic)>0)
    return NucleoInCodecMap[_topic];
  return NULL;
}

NucleoOutCodec* GetNucleoOutCodec(goldobot::CommMessageType _msg_id)
{
  if (NucleoOutCodecMap.count(_msg_id)>0)
    return NucleoOutCodecMap[_msg_id];
  return NULL;
}

void DebugDumpNucleoInCodec(void)
{
  cout << endl;
  cout << "NucleoInCodecMap:" << endl;
  for (map<std::string,NucleoInCodec*>::iterator it = NucleoInCodecMap.begin(); it != NucleoInCodecMap.end(); it++)
  {
    cout << "  " << it->first << " : " << it->second->my_name() << endl;
  }
}

void DebugDumpNucleoOutCodec(void)
{
  cout << endl;
  cout << "NucleoOutCodecMap:" << endl;
  for (map<goldobot::CommMessageType,NucleoOutCodec*>::iterator it = NucleoOutCodecMap.begin(); it != NucleoOutCodecMap.end(); it++)
  {
    cout << "  " << int(it->first) << " : " << it->second->my_name() << endl;
  }
}


#define NUCLEO_IN_CODEC(NAME, PB_TYPE, TOPIC, MSG_ID) class NucleoIn ## NAME ## Codec : public NucleoInCodec {             \
private:                                                                                                                   \
  PB_TYPE pb_msg;                                                                                                          \
public:                                                                                                                    \
  NucleoIn ## NAME ## Codec(const char *_topic, goldobot::CommMessageType _msg_id) : NucleoInCodec(_topic, _msg_id) {};    \
  ~NucleoIn ## NAME ## Codec() {};                                                                                         \
  virtual const char *my_name ()                                                                                           \
  {                                                                                                                        \
    return "NucleoIn" #NAME "Codec";                                                                                       \
  };                                                                                                                       \
  virtual const char *get_pb_desc_name ()                                                                                  \
  {                                                                                                                        \
    return pb_msg.GetDescriptor()->full_name().c_str();                                                                    \
  };                                                                                                                       \
  virtual const std::string& get_pb_desc_string ()                                                                         \
  {                                                                                                                        \
    return pb_msg.GetDescriptor()->full_name();                                                                            \
  };                                                                                                                       \
  virtual google::protobuf::Message* deserialize_pb (const std::string& pb_ser)                                            \
  {                                                                                                                        \
    pb_msg.Clear();                                                                                                        \
    pb_msg.ParseFromString(pb_ser);                                                                                        \
    return &pb_msg;                                                                                                        \
  };                                                                                                                       \
  virtual std::string& transcode_and_serialize_nucleo_msg ();                                                              \
};                                                                                                                         \
NucleoIn ## NAME ## Codec  NucleoIn ## NAME ( "nucleo/in/" TOPIC, MSG_ID )


#define NUCLEO_OUT_CODEC(NAME, PB_TYPE, TOPIC, MSG_ID) class NucleoOut ## NAME ## Codec : public NucleoOutCodec {          \
private:                                                                                                                   \
  PB_TYPE pb_msg;                                                                                                          \
public:                                                                                                                    \
  NucleoOut ## NAME ## Codec(const char *_topic, goldobot::CommMessageType _msg_id) : NucleoOutCodec(_topic, _msg_id) {};  \
  ~NucleoOut ## NAME ## Codec() {};                                                                                        \
  virtual const char *my_name ()                                                                                           \
  {                                                                                                                        \
    return "NucleoOut" #NAME "Codec";                                                                                      \
  };                                                                                                                       \
  virtual const char *get_pb_desc_name ()                                                                                  \
  {                                                                                                                        \
    return pb_msg.GetDescriptor()->full_name().c_str();                                                                    \
  };                                                                                                                       \
  virtual const std::string& get_pb_desc_string ()                                                                         \
  {                                                                                                                        \
    return pb_msg.GetDescriptor()->full_name();                                                                            \
  };                                                                                                                       \
  virtual google::protobuf::Message* transcode (const unsigned char* data_from_nucleo, int data_size);                     \
  virtual std::string& serialize_pb ()                                                                                     \
  {                                                                                                                        \
    nucleo_out_buf.clear();                                                                                                \
    pb_msg.SerializeToString(&nucleo_out_buf);                                                                             \
    return nucleo_out_buf;                                                                                                 \
  };                                                                                                                       \
};                                                                                                                         \
NucleoOut ## NAME ## Codec  NucleoOut ## NAME ( "nucleo/out/" TOPIC, MSG_ID )



NUCLEO_OUT_CODEC( OsHeartbeat , goldo::nucleo::Heartbeat , "os/heartbeat" , CommMessageType::Heartbeat );
google::protobuf::Message* NucleoOutOsHeartbeatCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned char *ptr = data_from_nucleo;
  const int32_t *heartbeat_val = (const int32_t *) ptr;

  pb_msg.set_timestamp(*heartbeat_val);

  return &pb_msg;
}

NUCLEO_IN_CODEC( OsPing , google::protobuf::Empty , "os/ping" , CommMessageType::CommUartPing );
std::string& NucleoInOsPingCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( OsPing , google::protobuf::Empty , "os/ping" , CommMessageType::CommUartPing );
google::protobuf::Message* NucleoOutOsPingCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsHeapStats , goldo::nucleo::FreeRTOSHeapStats , "os/heap_stats" , CommMessageType::HeapStats );
typedef struct xHeapStats
{
  size_t xAvailableHeapSpaceInBytes;      /* The total heap size currently available - this is the sum of all the free blocks, not the largest block that can be allocated. */
  size_t xSizeOfLargestFreeBlockInBytes;  /* The maximum size, in bytes, of all the free blocks within the heap at the time vPortGetHeapStats() is called. */
  size_t xSizeOfSmallestFreeBlockInBytes; /* The minimum size, in bytes, of all the free blocks within the heap at the time vPortGetHeapStats() is called. */
  size_t xNumberOfFreeBlocks;             /* The number of free memory blocks within the heap at the time vPortGetHeapStats() is called. */
  size_t xMinimumEverFreeBytesRemaining;  /* The minimum amount of total free memory (sum of all free blocks) there has been in the heap since the system booted. */
  size_t xNumberOfSuccessfulAllocations;  /* The number of calls to pvPortMalloc() that have returned a valid memory block. */
  size_t xNumberOfSuccessfulFrees;        /* The number of calls to vPortFree() that has successfully freed a block of memory. */
} HeapStats_t;
google::protobuf::Message* NucleoOutOsHeapStatsCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const HeapStats_t *my_stats = (const HeapStats_t *)data_from_nucleo;
  pb_msg.set_xavailableheapspaceinbytes(my_stats->xAvailableHeapSpaceInBytes);
  pb_msg.set_xsizeoflargestfreeblockinbytes(my_stats->xSizeOfLargestFreeBlockInBytes);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsTasksStats , goldo::nucleo::FreeRTOSTasksStats , "os/tasks_stats" , CommMessageType::TaskStats );
typedef struct xTaskStats {
  char task_name[16];
  uint32_t runtime_counter;
  uint16_t stack_high_watermark;
  uint16_t task_number;
} TaskStats_t;
google::protobuf::Message* NucleoOutOsTasksStatsCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  static const int MAX_TASKS=16;

  int n_tasks = data_size/24;
  if (n_tasks>MAX_TASKS) n_tasks = MAX_TASKS;

  pb_msg.Clear();

  for (int i=0; i<n_tasks; i++)
  {
    goldo::nucleo::FreeRTOSTaskStats *new_task_stats = pb_msg.add_tasks();
    const unsigned char *ptr = &data_from_nucleo[i*24];
    const TaskStats_t *nucleo_task_stats = (const TaskStats_t *) ptr;
    new_task_stats->set_task_name(nucleo_task_stats->task_name);
    new_task_stats->set_runtime_counter(nucleo_task_stats->runtime_counter);
    new_task_stats->set_task_number(nucleo_task_stats->task_number);
  }

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsDbgTrace , google::protobuf::BytesValue , "os/dbg_trace" , CommMessageType::DbgTrace );
/* FIXME : TODO : the "real" implementation of DbgTrace is an array of 64 "HalEventData_t".. */
typedef struct xHalEventData {
  uint32_t cycnt;
  uint8_t task_number;
  uint8_t interrupt_number;
  uint8_t event;
  uint8_t payload;
} HalEventData_t;
google::protobuf::Message* NucleoOutOsDbgTraceCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  pb_msg.set_value((const void*) data_from_nucleo, data_size);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsReset , google::protobuf::Empty , "os/reset", CommMessageType::Reset );
google::protobuf::Message* NucleoOutOsResetCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsTaskStatisticsUartComm , goldo::nucleo::statistics::UARTCommTaskStatistics , "os/task_statistics/uart_comm" , CommMessageType::UartCommTaskStatistics );
typedef struct xCommSerializer_Statistics {
  uint32_t messages_sent;
  uint32_t bytes_sent;
  uint32_t buffer_high_watermark;
} CommSerializer_Statistics_t;
typedef struct xCommDeserializer_Statistics {
  uint32_t messages_received;
  uint32_t bytes_received;
  uint32_t sequence_errors;
  uint32_t crc_errors;
  uint32_t buffer_high_watermark;
} CommDeserializer_Statistics_t;
typedef struct xMessageQueue_Statistics {
  uint32_t min_available_capacity;
  uint32_t bytes_pushed;
  uint32_t messages_pushed;
} MessageQueue_Statistics_t;
typedef struct xUARTCommTask_Statistics {
  uint32_t max_cycles;
  CommSerializer_Statistics_t serializer;
  CommDeserializer_Statistics_t deserializer;
  CommSerializer_Statistics_t serializer_ftdi;
  CommDeserializer_Statistics_t deserializer_ftdi;
  MessageQueue_Statistics_t out_queue;
  MessageQueue_Statistics_t out_prio_queue;
  MessageQueue_Statistics_t out_ftdi_queue;
} UARTCommTask_Statistics_t;
google::protobuf::Message* NucleoOutOsTaskStatisticsUartCommCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const UARTCommTask_Statistics_t *my_stats = (const UARTCommTask_Statistics_t *)data_from_nucleo;

  pb_msg.set_max_cycles(my_stats->max_cycles);

  goldo::nucleo::statistics::CommSerializerStatistics *serializer = 
      pb_msg.mutable_serializer();
  serializer->set_messages_sent(my_stats->serializer.messages_sent);
  serializer->set_bytes_sent(my_stats->serializer.bytes_sent);
  serializer->set_buffer_high_watermark(my_stats->serializer.buffer_high_watermark);

  goldo::nucleo::statistics::CommDeserializerStatistics *deserializer =
      pb_msg.mutable_deserializer();
  deserializer->set_messages_received(my_stats->deserializer.messages_received);
  deserializer->set_bytes_received(my_stats->deserializer.bytes_received);
  deserializer->set_sequence_errors(my_stats->deserializer.sequence_errors);
  deserializer->set_crc_errors(my_stats->deserializer.crc_errors);
  deserializer->set_buffer_high_watermark(my_stats->deserializer.buffer_high_watermark);

  goldo::nucleo::statistics::CommSerializerStatistics *serializer_ftdi =
      pb_msg.mutable_serializer_ftdi();
  serializer_ftdi->set_messages_sent(my_stats->serializer_ftdi.messages_sent);
  serializer_ftdi->set_bytes_sent(my_stats->serializer_ftdi.bytes_sent);
  serializer_ftdi->set_buffer_high_watermark(my_stats->serializer_ftdi.buffer_high_watermark);

  goldo::nucleo::statistics::CommDeserializerStatistics *deserializer_ftdi =
      pb_msg.mutable_deserializer_fdti();
  deserializer_ftdi->set_messages_received(my_stats->deserializer_ftdi.messages_received);
  deserializer_ftdi->set_bytes_received(my_stats->deserializer_ftdi.bytes_received);
  deserializer_ftdi->set_sequence_errors(my_stats->deserializer_ftdi.sequence_errors);
  deserializer_ftdi->set_crc_errors(my_stats->deserializer_ftdi.crc_errors);
  deserializer_ftdi->set_buffer_high_watermark(my_stats->deserializer_ftdi.buffer_high_watermark);

  goldo::nucleo::statistics::MessageQueueStatistics *queue_out =
      pb_msg.mutable_queue_out();
  queue_out->set_min_available_capacity(my_stats->out_queue.min_available_capacity);
  queue_out->set_bytes_pushed(my_stats->out_queue.bytes_pushed);
  queue_out->set_messages_pushed(my_stats->out_queue.messages_pushed);

  goldo::nucleo::statistics::MessageQueueStatistics *queue_out_prio = 
      pb_msg.mutable_queue_out_prio();
  queue_out_prio->set_min_available_capacity(my_stats->out_prio_queue.min_available_capacity);
  queue_out_prio->set_bytes_pushed(my_stats->out_prio_queue.bytes_pushed);
  queue_out_prio->set_messages_pushed(my_stats->out_prio_queue.messages_pushed);

  goldo::nucleo::statistics::MessageQueueStatistics *queue_out_ftdi =
      pb_msg.mutable_queue_out_ftdi();
  queue_out_ftdi->set_min_available_capacity(my_stats->out_ftdi_queue.min_available_capacity);
  queue_out_ftdi->set_bytes_pushed(my_stats->out_ftdi_queue.bytes_pushed);
  queue_out_ftdi->set_messages_pushed(my_stats->out_ftdi_queue.messages_pushed);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsTaskStatisticsOdriveComm , goldo::nucleo::statistics::ODriveCommTaskStatistics , "os/task_statistics/odrive_comm" , CommMessageType::ODriveCommTaskStatistics );
typedef struct xODriveStreamParser_Statistics {
  uint32_t bytes_received;
  uint16_t messages_received;
  uint16_t rx_errors;
} ODriveStreamParser_Statistics_t;
typedef struct xODriveStreamWriter_Statistics {
  uint32_t bytes_sent;
  uint16_t messages_sent;
  uint16_t tx_highwater;
} ODriveStreamWriter_Statistics_t;
typedef struct xODriveCommStats {
  ODriveStreamParser_Statistics_t parser;
  ODriveStreamWriter_Statistics_t writer;
  MessageQueue_Statistics_t queue;
} ODriveCommStats_t;
google::protobuf::Message* NucleoOutOsTaskStatisticsOdriveCommCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const ODriveCommStats_t *my_stats = (const ODriveCommStats_t *)data_from_nucleo;

  goldo::nucleo::statistics::ODriveStreamParserStatistics *parser = 
      pb_msg.mutable_parser();
  parser->set_bytes_received(my_stats->parser.bytes_received);
  parser->set_messages_received(my_stats->parser.messages_received);
  parser->set_rx_errors(my_stats->parser.rx_errors);

  goldo::nucleo::statistics::ODriveStreamWriterStatistics *writer = 
      pb_msg.mutable_writer();
  writer->set_bytes_sent(my_stats->writer.bytes_sent);
  writer->set_messages_sent(my_stats->writer.messages_sent);
  writer->set_tx_highwater(my_stats->writer.tx_highwater);

  goldo::nucleo::statistics::MessageQueueStatistics *queue = 
      pb_msg.mutable_queue();
  queue->set_min_available_capacity(my_stats->queue.min_available_capacity);
  queue->set_bytes_pushed(my_stats->queue.bytes_pushed);
  queue->set_messages_pushed(my_stats->queue.messages_pushed);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OsTaskStatisticsPropulsion , goldo::nucleo::statistics::PropulsionTaskStatistics , "os/task_statistics/propulsion" , CommMessageType::PropulsionTaskStatistics );
typedef struct xPropulsionTaskStatistics {
  uint32_t max_cycles;
  uint32_t min_interval;
  uint32_t max_interval;
  MessageQueue_Statistics_t queue;
  MessageQueue_Statistics_t urgent_queue;
  MessageQueue_Statistics_t odrive_queue;
} PropulsionTaskStatistics_t;
google::protobuf::Message* NucleoOutOsTaskStatisticsPropulsionCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const PropulsionTaskStatistics_t *my_stats = (const PropulsionTaskStatistics_t *)data_from_nucleo;

  pb_msg.set_max_cycles(my_stats->max_cycles);
  pb_msg.set_min_interval(my_stats->min_interval);
  pb_msg.set_max_interval(my_stats->max_interval);

  goldo::nucleo::statistics::MessageQueueStatistics *queue = 
      pb_msg.mutable_queue();
  queue->set_min_available_capacity(my_stats->queue.min_available_capacity);
  queue->set_bytes_pushed(my_stats->queue.bytes_pushed);
  queue->set_messages_pushed(my_stats->queue.messages_pushed);

  goldo::nucleo::statistics::MessageQueueStatistics *urgent_queue = 
      pb_msg.mutable_urgent_queue();
  urgent_queue->set_min_available_capacity(my_stats->urgent_queue.min_available_capacity);
  urgent_queue->set_bytes_pushed(my_stats->urgent_queue.bytes_pushed);
  urgent_queue->set_messages_pushed(my_stats->urgent_queue.messages_pushed);

  goldo::nucleo::statistics::MessageQueueStatistics *odrive_queue = 
      pb_msg.mutable_odrive_queue();
  odrive_queue->set_min_available_capacity(my_stats->odrive_queue.min_available_capacity);
  odrive_queue->set_bytes_pushed(my_stats->odrive_queue.bytes_pushed);
  odrive_queue->set_messages_pushed(my_stats->odrive_queue.messages_pushed);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( WatchdogState , goldo::nucleo::WatchdogState , "watchdog/state" , CommMessageType::WatchdogStatus );
typedef struct xWatchdogState {
  unsigned char main;
  unsigned char propulsion;
  unsigned char fpga;
  unsigned char odrive_comm;
  unsigned char dynamixels_comm;
  unsigned char servos;
} WatchdogState_t;
google::protobuf::Message* NucleoOutWatchdogStateCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const WatchdogState_t *my_state = (const WatchdogState_t *)data_from_nucleo;

  pb_msg.set_main(my_state->main!=0);
  pb_msg.set_propulsion(my_state->propulsion!=0);
  pb_msg.set_fpga(my_state->fpga!=0);
  pb_msg.set_odrive_comm(my_state->odrive_comm!=0);
  pb_msg.set_dynamixels_comm(my_state->dynamixels_comm!=0);
  pb_msg.set_servos(my_state->servos!=0);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( SensorsState , goldo::nucleo::SensorsState , "sensors/state" , CommMessageType::SensorsState );
typedef struct xSensorsState {
  uint32_t state;
} SensorsState_t;
google::protobuf::Message* NucleoOutSensorsStateCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const SensorsState_t *my_state = (const SensorsState_t *)data_from_nucleo;

  pb_msg.set_state(my_state->state);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( MatchTimer , google::protobuf::Int32Value , "match/timer" , CommMessageType::MatchTimer );
google::protobuf::Message* NucleoOutMatchTimerCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const uint32_t *my_timer = (const uint32_t *)data_from_nucleo;

  pb_msg.set_value(*my_timer);

  return &pb_msg;
}

NUCLEO_IN_CODEC( MatchTimerStart , google::protobuf::Empty , "match/timer/start" , CommMessageType::MatchTimerStart );
std::string& NucleoInMatchTimerStartCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( MatchTimerStop , google::protobuf::Empty , "match/timer/stop" , CommMessageType::MatchTimerStop );
std::string& NucleoInMatchTimerStopCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( MatchEnd , google::protobuf::Empty , "match/end" , CommMessageType::MatchEnd );
google::protobuf::Message* NucleoOutMatchEndCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  return &pb_msg;
}

NUCLEO_IN_CODEC( RobotConfigLoadBegin , goldo::nucleo::robot::ConfigLoadBegin , "robot/config/load_begin" , CommMessageType::RobotConfigLoadBegin );
std::string& NucleoInRobotConfigLoadBeginCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::robot::ConfigLoadBegin* load_begin_msg = (goldo::nucleo::robot::ConfigLoadBegin*) &pb_msg;
  uint16_t val = load_begin_msg->size();

  nucleo_in_buf.append((const char *)&val, sizeof(val));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( RobotConfigLoadChunk , goldo::nucleo::robot::ConfigLoadChunk , "robot/config/load_chunk" , CommMessageType::RobotConfigLoadChunk );
std::string& NucleoInRobotConfigLoadChunkCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::robot::ConfigLoadChunk* load_chunk_msg = (goldo::nucleo::robot::ConfigLoadChunk*) &pb_msg;

  nucleo_in_buf.append(load_chunk_msg->data());

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( RobotConfigLoadEnd , goldo::nucleo::robot::ConfigLoadEnd , "robot/config/load_end" , CommMessageType::RobotConfigLoadEnd );
std::string& NucleoInRobotConfigLoadEndCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::robot::ConfigLoadEnd* load_end_msg = (goldo::nucleo::robot::ConfigLoadEnd*) &pb_msg;
  uint16_t val = load_end_msg->crc();

  nucleo_in_buf.append((const char *)&val, sizeof(val));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( RobotConfigLoadStatus , goldo::nucleo::robot::ConfigLoadStatus , "robot/config/load_status" , CommMessageType::RobotConfigLoadStatus );
google::protobuf::Message* NucleoOutRobotConfigLoadStatusCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned char* my_status = (const unsigned char*) data_from_nucleo;

  pb_msg.set_status((goldo::nucleo::robot::ConfigLoadStatus_Status)*my_status);

  return &pb_msg;
}

NUCLEO_IN_CODEC( OdriveRequest , goldo::nucleo::odrive::RequestPacket , "odrive/request" , CommMessageType::ODriveRequestPacket );
std::string& NucleoInOdriveRequestCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::odrive::RequestPacket* request_packet_msg = (goldo::nucleo::odrive::RequestPacket*) &pb_msg;

  uint16_t sequence_number = request_packet_msg->sequence_number() & 0x3fff;
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint16_t endpoint_id = request_packet_msg->endpoint_id();
  nucleo_in_buf.append((const char *)&endpoint_id, sizeof(endpoint_id));

  uint16_t expected_response_size = request_packet_msg->expected_response_size();
  nucleo_in_buf.append((const char *)&expected_response_size, sizeof(expected_response_size));

  nucleo_in_buf.append(request_packet_msg->payload());

  uint16_t protocol_version = request_packet_msg->protocol_version();
  nucleo_in_buf.append((const char *)&protocol_version, sizeof(protocol_version));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( OdriveResponse , goldo::nucleo::odrive::ResponsePacket , "odrive/response" , CommMessageType::ODriveResponsePacket );
google::protobuf::Message* NucleoOutOdriveResponseCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned short* sequence_number = (const unsigned short*) data_from_nucleo;
  pb_msg.set_sequence_number(*sequence_number);

  const char* payload = (const char*) & data_from_nucleo[2];
  pb_msg.set_payload(payload, data_size-2);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( OdriveTelemetry , goldo::nucleo::odrive::Telemetry , "odrive/telemetry" , CommMessageType::ODriveTelemetry );
typedef struct xAxisTelemetry {
  float pos_estimate;
  float vel_estimate;
  float current_iq_setpoint;
} AxisTelemetry_t;
typedef struct xODriveClientTelemetry {
  uint32_t timestamp;
  AxisTelemetry_t axis[2];
} ODriveClientTelemetry_t;
google::protobuf::Message* NucleoOutOdriveTelemetryCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(ODriveClientTelemetry_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=ODriveTelemetry , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  ODriveClientTelemetry_t* my_telemetry = (ODriveClientTelemetry_t*) data_from_nucleo;

  pb_msg.set_timestamp(my_telemetry->timestamp);

  for (int i=0; i<2; i++)
  {
    goldo::nucleo::odrive::AxisTelemetry *new_axis = pb_msg.add_axis();
    new_axis->set_pos_estimate(my_telemetry->axis[i].pos_estimate);
    new_axis->set_vel_estimate(my_telemetry->axis[i].vel_estimate);
    new_axis->set_current_iq_setpoint(my_telemetry->axis[i].current_iq_setpoint);
  }

  return &pb_msg;
}

NUCLEO_IN_CODEC( DynamixelsRequest , goldo::nucleo::dynamixels::RequestPacket , "dynamixels/request" , CommMessageType::DynamixelsRequest );
std::string& NucleoInDynamixelsRequestCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::dynamixels::RequestPacket* request_packet_msg = (goldo::nucleo::dynamixels::RequestPacket*) &pb_msg;

  uint16_t sequence_number = request_packet_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  char protocol_version = request_packet_msg->protocol_version();
  nucleo_in_buf.append((const char *)&protocol_version, sizeof(protocol_version));

  char id = request_packet_msg->id();
  nucleo_in_buf.append((const char *)&id, sizeof(id));

  char command = request_packet_msg->command();
  nucleo_in_buf.append((const char *)&command, sizeof(command));

  nucleo_in_buf.append(request_packet_msg->payload());

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( DynamixelsResponse , goldo::nucleo::dynamixels::ResponsePacket , "dynamixels/response" , CommMessageType::DynamixelsResponse );
google::protobuf::Message* NucleoOutDynamixelsResponseCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  char *ptr = (char *) data_from_nucleo;
  size_t remain_sz = data_size;

  const unsigned short* sequence_number = (const unsigned short*) ptr;
  ptr += 2;
  remain_sz -= 2;
  pb_msg.set_sequence_number(*sequence_number);

  const unsigned char* protocol_version = (const unsigned char*) ptr;
  ptr += 1;
  remain_sz -= 1;
  pb_msg.set_protocol_version(*protocol_version);

  const unsigned char* id = (const unsigned char*) ptr;
  ptr += 1;
  remain_sz -= 1;
  pb_msg.set_id(*id);

  const unsigned char* error_flags = (const unsigned char*) ptr;
  ptr += 1;
  remain_sz -= 1;
  pb_msg.set_id(*error_flags);

  pb_msg.set_payload(ptr,remain_sz);

  return &pb_msg;
}

NUCLEO_IN_CODEC( FpgaRegRead , goldo::nucleo::fpga::RegRead , "fpga/reg/read" , CommMessageType::FpgaReadReg );
std::string& NucleoInFpgaRegReadCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::fpga::RegRead* fpga_reg_read_msg = (goldo::nucleo::fpga::RegRead*) &pb_msg;

  uint32_t apb_address = fpga_reg_read_msg->apb_address();
  nucleo_in_buf.append((const char *)&apb_address, sizeof(apb_address));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( FpgaReg , goldo::nucleo::fpga::RegReadStatus , "fpga/reg" , CommMessageType::FpgaReadRegStatus );
google::protobuf::Message* NucleoOutFpgaRegCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  char *ptr = (char *) data_from_nucleo;
  size_t remain_sz = data_size;

  const uint32_t* apb_address = (const uint32_t*) ptr;
  ptr += 4;
  remain_sz -= 4;
  pb_msg.set_apb_address(*apb_address);

  const uint32_t* apb_value = (const uint32_t*) ptr;
  ptr += 4;
  remain_sz -= 4;
  pb_msg.set_apb_value(*apb_value);

  return &pb_msg;
}

NUCLEO_IN_CODEC( FpgaRegWrite , goldo::nucleo::fpga::RegWrite , "fpga/reg/write" , CommMessageType::FpgaWriteReg );
std::string& NucleoInFpgaRegWriteCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::fpga::RegWrite* fpga_reg_write_msg = (goldo::nucleo::fpga::RegWrite*) &pb_msg;

  uint32_t apb_address = fpga_reg_write_msg->apb_address();
  nucleo_in_buf.append((const char *)&apb_address, sizeof(apb_address));

  uint32_t apb_value = fpga_reg_write_msg->apb_value();
  nucleo_in_buf.append((const char *)&apb_value, sizeof(apb_value));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( FpgaAdcRead , goldo::nucleo::fpga::AdcRead , "fpga/adc/read" , CommMessageType::FpgaReadAdc );
std::string& NucleoInFpgaAdcReadCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::fpga::AdcRead* fpga_adc_read_msg = (goldo::nucleo::fpga::AdcRead*) &pb_msg;

  uint32_t chan = fpga_adc_read_msg->chan();
  nucleo_in_buf.append((const char *)&chan, sizeof(chan));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( FpgaAdcReadOut , goldo::nucleo::fpga::AdcReadOut , "fpga/adc/read_out" , CommMessageType::FpgaReadAdcOut );
google::protobuf::Message* NucleoOutFpgaAdcReadOutCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  char *ptr = (char *) data_from_nucleo;
  size_t remain_sz = data_size;

  const uint32_t* chan = (const uint32_t*) ptr;
  ptr += 4;
  remain_sz -= 4;
  pb_msg.set_chan(*chan);

  const float* chan_val = (const float*) ptr;
  ptr += 4;
  remain_sz -= 4;
  pb_msg.set_chan_val(*chan_val);

  return &pb_msg;
}

NUCLEO_IN_CODEC( OdometryConfigGet , google::protobuf::Empty , "odometry/config/get" , CommMessageType::OdometryConfigGet );
std::string& NucleoInOdometryConfigGetCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( OdometryConfig , goldo::nucleo::odometry::OdometryConfig , "odometry/config" , CommMessageType::OdometryConfigGetStatus );
typedef struct xOdometryConfig {
  float dist_per_count_left;
  float dist_per_count_right;
  float wheel_distance_left;
  float wheel_distance_right;
  float speed_filter_frequency;
  float accel_filter_frequency;
} OdometryConfig_t;
google::protobuf::Message* NucleoOutOdometryConfigCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(OdometryConfig_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=OdometryConfigGetStatus , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  OdometryConfig_t* my_config = (OdometryConfig_t*) data_from_nucleo;

  pb_msg.set_dist_per_count_left(my_config->dist_per_count_left);
  pb_msg.set_dist_per_count_right(my_config->dist_per_count_right);
  pb_msg.set_wheel_distance_left(my_config->wheel_distance_left);
  pb_msg.set_wheel_distance_right(my_config->wheel_distance_right);
  pb_msg.set_speed_filter_frequency(my_config->speed_filter_frequency);
  pb_msg.set_accel_filter_frequency(my_config->accel_filter_frequency);

  return &pb_msg;
}

NUCLEO_IN_CODEC( OdometryConfigSet , goldo::nucleo::odometry::OdometryConfig , "odometry/config/set" , CommMessageType::OdometryConfigSet );
std::string& NucleoInOdometryConfigSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::odometry::OdometryConfig* odometry_config_set_msg = (goldo::nucleo::odometry::OdometryConfig*) &pb_msg;

  float dist_per_count_left = odometry_config_set_msg->dist_per_count_left();
  nucleo_in_buf.append((const char *)&dist_per_count_left, sizeof(dist_per_count_left));

  float dist_per_count_right = odometry_config_set_msg->dist_per_count_right();
  nucleo_in_buf.append((const char *)&dist_per_count_right, sizeof(dist_per_count_right));

  float wheel_distance_left = odometry_config_set_msg->wheel_distance_left();
  nucleo_in_buf.append((const char *)&wheel_distance_left, sizeof(wheel_distance_left));

  float wheel_distance_right = odometry_config_set_msg->wheel_distance_right();
  nucleo_in_buf.append((const char *)&wheel_distance_right, sizeof(wheel_distance_right));

  float speed_filter_frequency = odometry_config_set_msg->speed_filter_frequency();
  nucleo_in_buf.append((const char *)&speed_filter_frequency, sizeof(speed_filter_frequency));

  float accel_filter_frequency = odometry_config_set_msg->accel_filter_frequency();
  nucleo_in_buf.append((const char *)&accel_filter_frequency, sizeof(accel_filter_frequency));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionConfigGet , google::protobuf::Empty , "propulsion/config/get" , CommMessageType::PropulsionConfigGet );
std::string& NucleoInPropulsionConfigGetCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( PropulsionConfig , goldo::nucleo::propulsion::PropulsionControllerConfig , "propulsion/config" , CommMessageType::PropulsionConfigGetStatus );
typedef struct xPIDConfig {
  float kp;
  float ki;
  float kd;
  float lim_iterm;
  float lim_dterm;
  float d_filter_frequency;
  float min_output;
  float max_output;
} PIDConfig_t;
typedef struct xPropulsionLowLevelPIDConfig {
  PIDConfig_t speed_pid_config;
  PIDConfig_t longi_pid_config;
  PIDConfig_t yaw_rate_pid_config;
  PIDConfig_t yaw_pid_config;
} PropulsionLowLevelPIDConfig_t;
typedef struct xPropulsionLowLevelControllerConfig {
  float wheels_distance;      // distance between wheels
  float motors_speed_factor;  // conversion factor between speed in m/s at the wheels and motor
                              // velocity setpoint or pwm
} PropulsionLowLevelControllerConfig_t;
typedef struct xPropulsionControllerConfig {
  PropulsionLowLevelControllerConfig_t low_level_config;
  PropulsionLowLevelPIDConfig_t pid_configs[4];
  float lookahead_distance;
  float lookahead_time;
  float static_pwm_limit;
  float cruise_pwm_limit;
  float reposition_pwm_limit;
  float static_torque_limit;
  float cruise_torque_limit;
  float reposition_torque_limit;
} PropulsionControllerConfig_t;
google::protobuf::Message* NucleoOutPropulsionConfigCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(PropulsionControllerConfig_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionConfigGetStatus , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  PropulsionControllerConfig_t* my_config = (PropulsionControllerConfig_t*) data_from_nucleo;

  goldo::nucleo::propulsion::PropulsionLowLevelControllerConfig *low_level_config = 
    pb_msg.mutable_low_level_config();
  low_level_config->set_wheels_distance     (my_config->low_level_config.wheels_distance);
  low_level_config->set_motors_speed_factor (my_config->low_level_config.motors_speed_factor);

  for (int i=0; i<4; i++)
  {
    goldo::nucleo::propulsion::PropulsionLowLevelPIDConfig *new_pid_config = 
      pb_msg.add_pid_configs();

    goldo::nucleo::propulsion::PIDConfig *new_pid_config_speed = 
      new_pid_config->mutable_speed();
    new_pid_config_speed->set_kp                 (my_config->pid_configs[i].speed_pid_config.kp);
    new_pid_config_speed->set_ki                 (my_config->pid_configs[i].speed_pid_config.ki);
    new_pid_config_speed->set_kd                 (my_config->pid_configs[i].speed_pid_config.kd);
    new_pid_config_speed->set_lim_i              (my_config->pid_configs[i].speed_pid_config.lim_iterm);
    new_pid_config_speed->set_lim_d              (my_config->pid_configs[i].speed_pid_config.lim_dterm);
    new_pid_config_speed->set_d_filter_frequency (my_config->pid_configs[i].speed_pid_config.d_filter_frequency);
    new_pid_config_speed->set_out_min            (my_config->pid_configs[i].speed_pid_config.min_output);
    new_pid_config_speed->set_out_max            (my_config->pid_configs[i].speed_pid_config.max_output);

    goldo::nucleo::propulsion::PIDConfig *new_pid_config_longi = 
      new_pid_config->mutable_longi();
    new_pid_config_longi->set_kp                 (my_config->pid_configs[i].longi_pid_config.kp);
    new_pid_config_longi->set_ki                 (my_config->pid_configs[i].longi_pid_config.ki);
    new_pid_config_longi->set_kd                 (my_config->pid_configs[i].longi_pid_config.kd);
    new_pid_config_longi->set_lim_i              (my_config->pid_configs[i].longi_pid_config.lim_iterm);
    new_pid_config_longi->set_lim_d              (my_config->pid_configs[i].longi_pid_config.lim_dterm);
    new_pid_config_longi->set_d_filter_frequency (my_config->pid_configs[i].longi_pid_config.d_filter_frequency);
    new_pid_config_longi->set_out_min            (my_config->pid_configs[i].longi_pid_config.min_output);
    new_pid_config_longi->set_out_max            (my_config->pid_configs[i].longi_pid_config.max_output);

    goldo::nucleo::propulsion::PIDConfig *new_pid_config_yaw_rate = 
      new_pid_config->mutable_yaw_rate();
    new_pid_config_yaw_rate->set_kp                 (my_config->pid_configs[i].yaw_rate_pid_config.kp);
    new_pid_config_yaw_rate->set_ki                 (my_config->pid_configs[i].yaw_rate_pid_config.ki);
    new_pid_config_yaw_rate->set_kd                 (my_config->pid_configs[i].yaw_rate_pid_config.kd);
    new_pid_config_yaw_rate->set_lim_i              (my_config->pid_configs[i].yaw_rate_pid_config.lim_iterm);
    new_pid_config_yaw_rate->set_lim_d              (my_config->pid_configs[i].yaw_rate_pid_config.lim_dterm);
    new_pid_config_yaw_rate->set_d_filter_frequency (my_config->pid_configs[i].yaw_rate_pid_config.d_filter_frequency);
    new_pid_config_yaw_rate->set_out_min            (my_config->pid_configs[i].yaw_rate_pid_config.min_output);
    new_pid_config_yaw_rate->set_out_max            (my_config->pid_configs[i].yaw_rate_pid_config.max_output);

    goldo::nucleo::propulsion::PIDConfig *new_pid_config_yaw = 
      new_pid_config->mutable_yaw();
    new_pid_config_yaw->set_kp                 (my_config->pid_configs[i].yaw_pid_config.kp);
    new_pid_config_yaw->set_ki                 (my_config->pid_configs[i].yaw_pid_config.ki);
    new_pid_config_yaw->set_kd                 (my_config->pid_configs[i].yaw_pid_config.kd);
    new_pid_config_yaw->set_lim_i              (my_config->pid_configs[i].yaw_pid_config.lim_iterm);
    new_pid_config_yaw->set_lim_d              (my_config->pid_configs[i].yaw_pid_config.lim_dterm);
    new_pid_config_yaw->set_d_filter_frequency (my_config->pid_configs[i].yaw_pid_config.d_filter_frequency);
    new_pid_config_yaw->set_out_min            (my_config->pid_configs[i].yaw_pid_config.min_output);
    new_pid_config_yaw->set_out_max            (my_config->pid_configs[i].yaw_pid_config.max_output);
  }

  pb_msg.set_lookahead_distance           (my_config->lookahead_distance);
  pb_msg.set_lookahead_time               (my_config->lookahead_time);
  pb_msg.set_static_motor_speed_limit     (my_config->static_pwm_limit);
  pb_msg.set_cruise_motor_speed_limit     (my_config->cruise_pwm_limit);
  pb_msg.set_reposition_motor_speed_limit (my_config->reposition_pwm_limit);
  pb_msg.set_static_torque_limit          (my_config->static_torque_limit);
  pb_msg.set_cruise_torque_limit          (my_config->cruise_torque_limit);
  pb_msg.set_reposition_torque_limit      (my_config->reposition_torque_limit);

  return &pb_msg;
}

NUCLEO_IN_CODEC( PropulsionConfigSet , goldo::nucleo::propulsion::PropulsionControllerConfig , "propulsion/config/set" , CommMessageType::PropulsionConfigSet );
std::string& NucleoInPropulsionConfigSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::PropulsionControllerConfig* nucleo_in_propulsion_config_set_msg = (goldo::nucleo::propulsion::PropulsionControllerConfig*) &pb_msg;

  PropulsionLowLevelControllerConfig_t low_level_config;
  low_level_config.wheels_distance =     nucleo_in_propulsion_config_set_msg->low_level_config().wheels_distance();
  low_level_config.motors_speed_factor = nucleo_in_propulsion_config_set_msg->low_level_config().motors_speed_factor();
  nucleo_in_buf.append((const char *)&low_level_config, sizeof(low_level_config));

  PropulsionLowLevelPIDConfig_t pid_configs[4];
  for (int i=0; i<4; i++)
  {
    pid_configs[i].speed_pid_config.kp                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().kp();
    pid_configs[i].speed_pid_config.ki                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().ki();
    pid_configs[i].speed_pid_config.kd                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().kd();
    pid_configs[i].speed_pid_config.lim_iterm             = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().lim_i();
    pid_configs[i].speed_pid_config.lim_dterm             = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().lim_d();
    pid_configs[i].speed_pid_config.d_filter_frequency    = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().d_filter_frequency();
    pid_configs[i].speed_pid_config.min_output            = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().out_min();
    pid_configs[i].speed_pid_config.max_output            = nucleo_in_propulsion_config_set_msg->pid_configs(i).speed().out_max();

    pid_configs[i].longi_pid_config.kp                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().kp();
    pid_configs[i].longi_pid_config.ki                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().ki();
    pid_configs[i].longi_pid_config.kd                    = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().kd();
    pid_configs[i].longi_pid_config.lim_iterm             = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().lim_i();
    pid_configs[i].longi_pid_config.lim_dterm             = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().lim_d();
    pid_configs[i].longi_pid_config.d_filter_frequency    = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().d_filter_frequency();
    pid_configs[i].longi_pid_config.min_output            = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().out_min();
    pid_configs[i].longi_pid_config.max_output            = nucleo_in_propulsion_config_set_msg->pid_configs(i).longi().out_max();

    pid_configs[i].yaw_rate_pid_config.kp                 = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().kp();
    pid_configs[i].yaw_rate_pid_config.ki                 = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().ki();
    pid_configs[i].yaw_rate_pid_config.kd                 = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().kd();
    pid_configs[i].yaw_rate_pid_config.lim_iterm          = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().lim_i();
    pid_configs[i].yaw_rate_pid_config.lim_dterm          = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().lim_d();
    pid_configs[i].yaw_rate_pid_config.d_filter_frequency = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().d_filter_frequency();
    pid_configs[i].yaw_rate_pid_config.min_output         = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().out_min();
    pid_configs[i].yaw_rate_pid_config.max_output         = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw_rate().out_max();

    pid_configs[i].yaw_pid_config.kp                      = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().kp();
    pid_configs[i].yaw_pid_config.ki                      = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().ki();
    pid_configs[i].yaw_pid_config.kd                      = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().kd();
    pid_configs[i].yaw_pid_config.lim_iterm               = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().lim_i();
    pid_configs[i].yaw_pid_config.lim_dterm               = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().lim_d();
    pid_configs[i].yaw_pid_config.d_filter_frequency      = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().d_filter_frequency();
    pid_configs[i].yaw_pid_config.min_output              = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().out_min();
    pid_configs[i].yaw_pid_config.max_output              = nucleo_in_propulsion_config_set_msg->pid_configs(i).yaw().out_max();
  }
  nucleo_in_buf.append((const char *)&pid_configs, sizeof(pid_configs));

  float lookahead_distance         = nucleo_in_propulsion_config_set_msg->lookahead_distance();
  nucleo_in_buf.append((const char *)&lookahead_distance, sizeof(lookahead_distance));

  float lookahead_time             = nucleo_in_propulsion_config_set_msg->lookahead_time();
  nucleo_in_buf.append((const char *)&lookahead_time, sizeof(lookahead_time));

  float static_pwm_limit           = nucleo_in_propulsion_config_set_msg->static_motor_speed_limit();
  nucleo_in_buf.append((const char *)&static_pwm_limit, sizeof(static_pwm_limit));

  float cruise_pwm_limit           = nucleo_in_propulsion_config_set_msg->cruise_motor_speed_limit();
  nucleo_in_buf.append((const char *)&cruise_pwm_limit, sizeof(cruise_pwm_limit));

  float reposition_pwm_limit       = nucleo_in_propulsion_config_set_msg->reposition_motor_speed_limit();
  nucleo_in_buf.append((const char *)&reposition_pwm_limit, sizeof(reposition_pwm_limit));

  float static_torque_limit        = nucleo_in_propulsion_config_set_msg->static_torque_limit();
  nucleo_in_buf.append((const char *)&static_torque_limit, sizeof(static_torque_limit));

  float cruise_torque_limit        = nucleo_in_propulsion_config_set_msg->cruise_torque_limit();
  nucleo_in_buf.append((const char *)&cruise_torque_limit, sizeof(cruise_torque_limit));

  float reposition_torque_limit    = nucleo_in_propulsion_config_set_msg->reposition_torque_limit();
  nucleo_in_buf.append((const char *)&reposition_torque_limit, sizeof(reposition_torque_limit));

  return nucleo_in_buf;
}
  
NUCLEO_IN_CODEC( PropulsionEnableSet , goldo::nucleo::propulsion::CmdSetEnable , "propulsion/enable/set" , CommMessageType::PropulsionEnableSet );
std::string& NucleoInPropulsionEnableSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdSetEnable* nucleo_in_propulsion_enable_set_msg = (goldo::nucleo::propulsion::CmdSetEnable*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_enable_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint8_t enable = (int8_t) nucleo_in_propulsion_enable_set_msg->enable();
  nucleo_in_buf.append((const char *)&enable, sizeof(enable));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionSimulationEnable , google::protobuf::BoolValue , "propulsion/simulation/enable" , CommMessageType::PropulsionSetSimulationMode );
std::string& NucleoInPropulsionSimulationEnableCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  google::protobuf::BoolValue* nucleo_in_propulsion_enable_msg = (google::protobuf::BoolValue*) &pb_msg;

  uint8_t enable = (uint8_t) nucleo_in_propulsion_enable_msg->value();
  nucleo_in_buf.append((const char *)&enable, sizeof(enable));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionMotorsEnableSet , google::protobuf::BoolValue , "propulsion/motors/enable/set" , CommMessageType::PropulsionMotorsEnableSet );
std::string& NucleoInPropulsionMotorsEnableSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  google::protobuf::BoolValue* nucleo_in_propulsion_motors_enable_set_msg = (google::protobuf::BoolValue*) &pb_msg;

  uint8_t enable = (uint8_t) nucleo_in_propulsion_motors_enable_set_msg->value();
  nucleo_in_buf.append((const char *)&enable, sizeof(enable));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionEmergencyStop , goldo::nucleo::propulsion::CmdEmpty , "propulsion/emergency_stop" , CommMessageType::PropulsionEmergencyStop );
std::string& NucleoInPropulsionEmergencyStopCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdEmpty* nucleo_in_propulsion_emergency_stop_msg =
    (goldo::nucleo::propulsion::CmdEmpty*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_emergency_stop_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionClearError , goldo::nucleo::propulsion::CmdEmpty , "propulsion/clear_error" , CommMessageType::PropulsionClearError );
std::string& NucleoInPropulsionClearErrorCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdEmpty* nucleo_in_propulsion_clear_error_msg =
    (goldo::nucleo::propulsion::CmdEmpty*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_clear_error_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionMotorsVelocitySetpointsSet , goldo::nucleo::propulsion::MotorsVelocitySetpoints , "propulsion/motors/velocity_setpoints/set" , CommMessageType::PropulsionMotorsVelocitySetpointsSet );
std::string& NucleoInPropulsionMotorsVelocitySetpointsSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::MotorsVelocitySetpoints* nucleo_in_propulsion_motors_velocity_setpoints_set_msg = (goldo::nucleo::propulsion::MotorsVelocitySetpoints*) &pb_msg;

  float left_vel = (float) nucleo_in_propulsion_motors_velocity_setpoints_set_msg->left_vel();
  nucleo_in_buf.append((const char *)&left_vel, sizeof(left_vel));

  float right_vel = (float) nucleo_in_propulsion_motors_velocity_setpoints_set_msg->right_vel();
  nucleo_in_buf.append((const char *)&right_vel, sizeof(right_vel));

  float left_current_feedforward = (float) nucleo_in_propulsion_motors_velocity_setpoints_set_msg->left_current_feedforward();
  nucleo_in_buf.append((const char *)&left_current_feedforward, sizeof(left_current_feedforward));

  float right_current_feedforward = (float) nucleo_in_propulsion_motors_velocity_setpoints_set_msg->right_current_feedforward();
  nucleo_in_buf.append((const char *)&right_current_feedforward, sizeof(right_current_feedforward));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionTargetSpeedSet , goldo::nucleo::propulsion::CmdSetTargetSpeed , "propulsion/target_speed/set" , CommMessageType::PropulsionSetTargetSpeed );
std::string& NucleoInPropulsionTargetSpeedSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdSetTargetSpeed* nucleo_in_propulsion_target_speed_set_msg =
    (goldo::nucleo::propulsion::CmdSetTargetSpeed*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_target_speed_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float target_speed = (float) nucleo_in_propulsion_target_speed_set_msg->target_speed();
  nucleo_in_buf.append((const char *)&target_speed, sizeof(target_speed));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionPoseSet , goldo::nucleo::propulsion::CmdSetPose , "propulsion/pose/set" , CommMessageType::PropulsionSetPose );
std::string& NucleoInPropulsionPoseSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdSetPose* nucleo_in_propulsion_pose_set_msg = (goldo::nucleo::propulsion::CmdSetPose*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_pose_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float position_x = (float) nucleo_in_propulsion_pose_set_msg->position().x();
  nucleo_in_buf.append((const char *)&position_x, sizeof(position_x));

  float position_y = (float) nucleo_in_propulsion_pose_set_msg->position().y();
  nucleo_in_buf.append((const char *)&position_y, sizeof(position_y));

  float yaw = (float) nucleo_in_propulsion_pose_set_msg->yaw();
  nucleo_in_buf.append((const char *)&yaw, sizeof(yaw));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCalibrateOdrive , google::protobuf::Empty , "propulsion/calibrate_odrive" , CommMessageType::PropulsionCalibrateODrive );
std::string& NucleoInPropulsionCalibrateOdriveCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionOdriveClearErrors , google::protobuf::Empty , "propulsion/odrive/clear_errors" , CommMessageType::PropulsionODriveClearErrors );
std::string& NucleoInPropulsionOdriveClearErrorsCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( PropulsionOdriveStatistics , goldo::nucleo::odrive::ClientStatistics , "propulsion/odrive/statistics" , CommMessageType::PropulsionODriveStatistics );
typedef struct xODriveStats {
  uint16_t max_latency;
  uint16_t timeout_errors;
  uint32_t uptime;
  uint8_t  synchronized;
} ODriveStats_t;
google::protobuf::Message* NucleoOutPropulsionOdriveStatisticsCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(ODriveStats_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionODriveStatistics , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  ODriveStats_t* my_stats = (ODriveStats_t*) data_from_nucleo;

  pb_msg.set_max_latency    (my_stats->max_latency);
  pb_msg.set_timeout_errors (my_stats->timeout_errors);
  pb_msg.set_uptime         (my_stats->uptime);
  pb_msg.set_synchronized   (my_stats->synchronized!=0);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionOdriveAxisStates , goldo::nucleo::odrive::AxisStates , "propulsion/odrive/axis_states" , CommMessageType::PropulsionODriveAxisStates );
typedef struct xAxisState {
  uint32_t current_state;
  uint32_t requested_state;
  uint32_t control_mode;
} AxisState_t;
typedef struct xOdriveAxisStates {
  AxisState_t axis0;
  AxisState_t axis1;
} OdriveAxisStates_t;
google::protobuf::Message* NucleoOutPropulsionOdriveAxisStatesCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(OdriveAxisStates_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionODriveAxisStates , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  OdriveAxisStates_t* my_states = (OdriveAxisStates_t*) data_from_nucleo;

  pb_msg.mutable_axis0()->set_current_state   ((goldo::nucleo::odrive::AxisState) my_states->axis0.current_state);
  pb_msg.mutable_axis0()->set_requested_state ((goldo::nucleo::odrive::AxisState) my_states->axis0.requested_state);
  pb_msg.mutable_axis0()->set_control_mode    ((goldo::nucleo::odrive::ControlMode)my_states->axis0.control_mode);

  pb_msg.mutable_axis1()->set_current_state   ((goldo::nucleo::odrive::AxisState) my_states->axis1.current_state);
  pb_msg.mutable_axis1()->set_requested_state ((goldo::nucleo::odrive::AxisState) my_states->axis1.requested_state);
  pb_msg.mutable_axis1()->set_control_mode    ((goldo::nucleo::odrive::ControlMode)my_states->axis1.control_mode);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionOdriveErrors , goldo::nucleo::odrive::AxisErrorStates , "propulsion/odrive/errors" , CommMessageType::PropulsionODriveAxisErrors );
typedef struct xAxisErrorState {
  uint32_t axis;
  uint32_t motor;
  uint32_t controller;
  uint32_t encoder;
  uint32_t sensorless_estimator;
} AxisErrorState_t;
typedef struct xOdriveAxisErrorStates {
  AxisErrorState_t axis0;
  AxisErrorState_t axis1;
} OdriveAxisErrorStates_t;
google::protobuf::Message* NucleoOutPropulsionOdriveErrorsCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(OdriveAxisErrorStates_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionODriveAxisErrors , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  OdriveAxisErrorStates_t* my_errors = (OdriveAxisErrorStates_t*) data_from_nucleo;

  pb_msg.mutable_axis0()->set_axis                 (my_errors->axis0.axis);
  pb_msg.mutable_axis0()->set_motor                (my_errors->axis0.motor);
  pb_msg.mutable_axis0()->set_controller           (my_errors->axis0.controller);
  pb_msg.mutable_axis0()->set_encoder              (my_errors->axis0.encoder);
  pb_msg.mutable_axis0()->set_sensorless_estimator (my_errors->axis0.sensorless_estimator);

  pb_msg.mutable_axis1()->set_axis                 (my_errors->axis1.axis);
  pb_msg.mutable_axis1()->set_motor                (my_errors->axis1.motor);
  pb_msg.mutable_axis1()->set_controller           (my_errors->axis1.controller);
  pb_msg.mutable_axis1()->set_encoder              (my_errors->axis1.encoder);
  pb_msg.mutable_axis1()->set_sensorless_estimator (my_errors->axis1.sensorless_estimator);

  return &pb_msg;
}

NUCLEO_IN_CODEC( PropulsionClearCommandQueue , google::protobuf::Empty , "propulsion/clear_command_queue" , CommMessageType::PropulsionClearCommandQueue );
std::string& NucleoInPropulsionClearCommandQueueCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdTranslation , goldo::nucleo::propulsion::ExecuteTranslation , "propulsion/cmd/translation" , CommMessageType::PropulsionExecuteTranslation );
std::string& NucleoInPropulsionCmdTranslationCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteTranslation* nucleo_in_propulsion_cmd_translation_msg =
    (goldo::nucleo::propulsion::ExecuteTranslation*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_translation_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float distance = (float) nucleo_in_propulsion_cmd_translation_msg->distance();
  nucleo_in_buf.append((const char *)&distance, sizeof(distance));

  float speed = (float) nucleo_in_propulsion_cmd_translation_msg->speed();
  nucleo_in_buf.append((const char *)&speed, sizeof(speed));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdReposition , goldo::nucleo::propulsion::ExecuteReposition , "propulsion/cmd/reposition" , CommMessageType::PropulsionExecuteReposition );
std::string& NucleoInPropulsionCmdRepositionCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteReposition* nucleo_in_propulsion_cmd_reposition_msg =
    (goldo::nucleo::propulsion::ExecuteReposition*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_reposition_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float distance = (float) nucleo_in_propulsion_cmd_reposition_msg->distance();
  nucleo_in_buf.append((const char *)&distance, sizeof(distance));

  float speed = (float) nucleo_in_propulsion_cmd_reposition_msg->speed();
  nucleo_in_buf.append((const char *)&speed, sizeof(speed));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdMeasureNormal , goldo::nucleo::propulsion::CmdMeasureNormal , "propulsion/cmd/measure_normal" , CommMessageType::PropulsionMeasureNormal );
std::string& NucleoInPropulsionCmdMeasureNormalCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdMeasureNormal* nucleo_in_propulsion_cmd_measure_normal_msg =
    (goldo::nucleo::propulsion::CmdMeasureNormal*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_measure_normal_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float angle = (float) nucleo_in_propulsion_cmd_measure_normal_msg->angle();
  nucleo_in_buf.append((const char *)&angle, sizeof(angle));

  float distance = (float) nucleo_in_propulsion_cmd_measure_normal_msg->distance();
  nucleo_in_buf.append((const char *)&distance, sizeof(distance));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdMoveTo , goldo::nucleo::propulsion::ExecuteMoveTo , "propulsion/cmd/move_to" , CommMessageType::PropulsionExecuteMoveTo );
std::string& NucleoInPropulsionCmdMoveToCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteMoveTo* nucleo_in_propulsion_cmd_move_to_msg =
    (goldo::nucleo::propulsion::ExecuteMoveTo*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_move_to_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float position_x = (float) nucleo_in_propulsion_cmd_move_to_msg->point().x();
  nucleo_in_buf.append((const char *)&position_x, sizeof(position_x));

  float position_y = (float) nucleo_in_propulsion_cmd_move_to_msg->point().y();
  nucleo_in_buf.append((const char *)&position_y, sizeof(position_y));

  float speed = (float) nucleo_in_propulsion_cmd_move_to_msg->speed();
  nucleo_in_buf.append((const char *)&speed, sizeof(speed));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdRotation , goldo::nucleo::propulsion::ExecuteRotation , "propulsion/cmd/rotation" , CommMessageType::PropulsionExecuteRotation );
std::string& NucleoInPropulsionCmdRotationCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteRotation* nucleo_in_propulsion_cmd_rotation_msg =
    (goldo::nucleo::propulsion::ExecuteRotation*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_rotation_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float angle = (float) nucleo_in_propulsion_cmd_rotation_msg->angle();
  nucleo_in_buf.append((const char *)&angle, sizeof(angle));

  float yaw_rate = (float) nucleo_in_propulsion_cmd_rotation_msg->yaw_rate();
  nucleo_in_buf.append((const char *)&yaw_rate, sizeof(yaw_rate));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdPointTo , goldo::nucleo::propulsion::ExecutePointTo , "propulsion/cmd/point_to" , CommMessageType::PropulsionExecutePointTo );
std::string& NucleoInPropulsionCmdPointToCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecutePointTo* nucleo_in_propulsion_cmd_point_to_msg =
    (goldo::nucleo::propulsion::ExecutePointTo*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_point_to_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float position_x = (float) nucleo_in_propulsion_cmd_point_to_msg->point().x();
  nucleo_in_buf.append((const char *)&position_x, sizeof(position_x));

  float position_y = (float) nucleo_in_propulsion_cmd_point_to_msg->point().y();
  nucleo_in_buf.append((const char *)&position_y, sizeof(position_y));

  float yaw_rate = (float) nucleo_in_propulsion_cmd_point_to_msg->yaw_rate();
  nucleo_in_buf.append((const char *)&yaw_rate, sizeof(yaw_rate));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdPointToBack , goldo::nucleo::propulsion::ExecutePointTo , "propulsion/cmd/point_to_back" , CommMessageType::PropulsionExecutePointToBack );
std::string& NucleoInPropulsionCmdPointToBackCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecutePointTo* nucleo_in_propulsion_cmd_point_to_back_msg =
    (goldo::nucleo::propulsion::ExecutePointTo*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_point_to_back_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float position_x = (float) nucleo_in_propulsion_cmd_point_to_back_msg->point().x();
  nucleo_in_buf.append((const char *)&position_x, sizeof(position_x));

  float position_y = (float) nucleo_in_propulsion_cmd_point_to_back_msg->point().y();
  nucleo_in_buf.append((const char *)&position_y, sizeof(position_y));

  float yaw_rate = (float) nucleo_in_propulsion_cmd_point_to_back_msg->yaw_rate();
  nucleo_in_buf.append((const char *)&yaw_rate, sizeof(yaw_rate));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdFaceDirection , goldo::nucleo::propulsion::ExecuteFaceDirection , "propulsion/cmd/face_direction" , CommMessageType::PropulsionExecuteFaceDirection );
std::string& NucleoInPropulsionCmdFaceDirectionCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteFaceDirection* nucleo_in_propulsion_cmd_face_direction_msg =
    (goldo::nucleo::propulsion::ExecuteFaceDirection*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_face_direction_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float yaw = (float) nucleo_in_propulsion_cmd_face_direction_msg->yaw();
  nucleo_in_buf.append((const char *)&yaw, sizeof(yaw));

  float yaw_rate = (float) nucleo_in_propulsion_cmd_face_direction_msg->yaw_rate();
  nucleo_in_buf.append((const char *)&yaw_rate, sizeof(yaw_rate));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( PropulsionCmdEvent , goldo::nucleo::propulsion::CommandStatus , "propulsion/cmd_event" , CommMessageType::PropulsionCommandEvent );
typedef struct xCmdEvent {
  uint32_t timestamp;
  uint16_t sequence_number;
  uint8_t  status;
  uint8_t  error;
} CmdEvent_t;
google::protobuf::Message* NucleoOutPropulsionCmdEventCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(CmdEvent_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionCommandEvent , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  CmdEvent_t* my_event = (CmdEvent_t*) data_from_nucleo;

  pb_msg.set_timestamp(my_event->timestamp);
  pb_msg.set_sequence_number(my_event->sequence_number);
  pb_msg.set_status(my_event->status);
  pb_msg.set_error(my_event->error);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionTelemetry , goldo::nucleo::propulsion::Telemetry , "propulsion/telemetry" , CommMessageType::PropulsionTelemetry );
typedef struct xPropulsionTelemetry {
  int16_t x;  // quarters of mm
  int16_t y;
  int16_t yaw;
  int16_t speed;     // mm per second
  int16_t yaw_rate;  // mradian per second
  int16_t acceleration;
  int16_t angular_acceleration;
  uint16_t left_encoder;
  uint16_t right_encoder;
  int8_t left_pwm;  // percents
  int8_t right_pwm;
  uint8_t state;
  uint8_t error;
} PropulsionTelemetry_t;
google::protobuf::Message* NucleoOutPropulsionTelemetryCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(PropulsionTelemetry_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionTelemetry , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  PropulsionTelemetry_t* my_telemetry = (PropulsionTelemetry_t*) data_from_nucleo;

  pb_msg.mutable_pose()->mutable_position()->set_x ((float)my_telemetry->x * 0.25e-3);
  pb_msg.mutable_pose()->mutable_position()->set_y ((float)my_telemetry->y * 0.25e-3);
  pb_msg.mutable_pose()->set_yaw                   ((float)my_telemetry->yaw * M_PI / 32767);
  pb_msg.mutable_pose()->set_speed                 ((float)my_telemetry->speed * 1e-3);
  pb_msg.mutable_pose()->set_yaw_rate              ((float)my_telemetry->yaw_rate * 1e-3);
  pb_msg.mutable_pose()->set_acceleration          ((float)my_telemetry->acceleration * 1e-3);
  pb_msg.mutable_pose()->set_angular_acceleration  ((float)my_telemetry->angular_acceleration * 1e-3);
  pb_msg.set_left_encoder                          ((float)my_telemetry->left_encoder);
  pb_msg.set_right_encoder                         ((float)my_telemetry->right_encoder);
  pb_msg.set_left_pwm                              ((float)my_telemetry->left_pwm * 1e-2);
  pb_msg.set_right_pwm                             ((float)my_telemetry->right_pwm * 1e-2);
  pb_msg.set_state                                 ((goldo::nucleo::propulsion::PropulsionControllerState)my_telemetry->state);
  pb_msg.set_error                                 (my_telemetry->error);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionTelemetryEx , goldo::nucleo::propulsion::TelemetryEx , "propulsion/telemetry_ex" , CommMessageType::PropulsionTelemetryEx );
typedef struct xPropulsionTelemetryEx {
  int16_t target_x;  // quarters of mm
  int16_t target_y;
  int16_t target_yaw;
  int16_t target_speed;     // mm per second
  int16_t target_yaw_rate;  // mradian per second
  int16_t lookahead_x;      // quarters of mm
  int16_t lookahead_y;
  int16_t longitudinal_error;
  int16_t lateral_error;
  int16_t speed_error;
  int16_t yaw_error;
  int16_t yaw_rate_error;
} PropulsionTelemetryEx_t;
google::protobuf::Message* NucleoOutPropulsionTelemetryExCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(PropulsionTelemetryEx_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionTelemetryEx , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  PropulsionTelemetryEx_t* my_telemetry_ex = (PropulsionTelemetryEx_t*) data_from_nucleo;

  pb_msg.mutable_target_pose()->mutable_position()->set_x  ((float)my_telemetry_ex->target_x * 0.25e-3);
  pb_msg.mutable_target_pose()->mutable_position()->set_y  ((float)my_telemetry_ex->target_y * 0.25e-3);
  pb_msg.mutable_target_pose()->set_yaw                    ((float)my_telemetry_ex->target_yaw * M_PI / 32767);
  pb_msg.mutable_target_pose()->set_speed                  ((float)my_telemetry_ex->target_speed * 1e-3);
  pb_msg.mutable_target_pose()->set_yaw_rate               ((float)my_telemetry_ex->target_yaw_rate * 1e-3);
  pb_msg.mutable_target_pose()->set_acceleration           (0.0);
  pb_msg.mutable_target_pose()->set_angular_acceleration   (0.0);
  pb_msg.mutable_lookahead_position()->set_x               ((float)my_telemetry_ex->lookahead_x * 0.25e-3);
  pb_msg.mutable_lookahead_position()->set_y               ((float)my_telemetry_ex->lookahead_y * 0.25e-3);
  pb_msg.set_error_longi                                   ((float)my_telemetry_ex->longitudinal_error);
  pb_msg.set_error_lateral                                 ((float)my_telemetry_ex->lateral_error);
  pb_msg.set_error_speed                                   ((float)my_telemetry_ex->speed_error);
  pb_msg.set_error_yaw                                     ((float)my_telemetry_ex->yaw_error);
  pb_msg.set_error_yaw_rate                                ((float)my_telemetry_ex->yaw_rate_error);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionState , goldo::nucleo::propulsion::StateChange , "propulsion/state" , CommMessageType::PropulsionState );
google::protobuf::Message* NucleoOutPropulsionStateCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  char *ptr = (char *) data_from_nucleo;
  size_t remain_sz = data_size;

  const uint32_t* timestamp = (const uint32_t*) ptr;
  ptr += 4;
  remain_sz -= 4;
  pb_msg.set_timestamp(*timestamp);

  const uint8_t* state = (const uint8_t*) ptr;
  ptr += 1;
  remain_sz -= 1;
  pb_msg.set_state((goldo::nucleo::propulsion::PropulsionControllerState)*state);

  const uint8_t* error = (const uint8_t*) ptr;
  ptr += 1;
  remain_sz -= 1;
  pb_msg.set_error((goldo::nucleo::propulsion::PropulsionControllerError)*error);

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionOdriveTelemetry , goldo::nucleo::odrive::Telemetry , "propulsion/odrive/telemetry" , CommMessageType::PropulsionODriveTelemetry );
google::protobuf::Message* NucleoOutPropulsionOdriveTelemetryCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  goldo::nucleo::odrive::Telemetry *t_msg = (goldo::nucleo::odrive::Telemetry *) NucleoOutOdriveTelemetry.transcode(data_from_nucleo, data_size);

  pb_msg = *t_msg; /* FIXME : TODO : TEST : really a deep-copy??.. */

  return &pb_msg;
}

NUCLEO_OUT_CODEC( PropulsionOdometryStream , google::protobuf::BytesValue , "propulsion/odometry_stream" , CommMessageType::PropulsionOdometryStream );
google::protobuf::Message* NucleoOutPropulsionOdometryStreamCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  pb_msg.set_value((const void*) data_from_nucleo, data_size);

  return &pb_msg;
}

/* @nucleo_in('propulsion/scope/config/set', nucleo_message_ids.PropulsionScopeConfig) */
/* FIXME : TODO */

/* @nucleo_out('propulsion/scope/data', nucleo_message_ids.PropulsionScopeData) */
/* FIXME : TODO */

NUCLEO_OUT_CODEC( PropulsionControllerEvent , goldo::nucleo::propulsion::PropulsionEvent , "propulsion/controller/event" , CommMessageType::PropulsionControllerEvent );
typedef struct xControllerEvent {
  float    pose_x;
  float    pose_y;
  float    pose_yaw;
  float    pose_speed;
  float    pose_yaw_rate;
  float    pose_acceleration;
  float    pose_angular_acceleration;
  float    parameter;
  uint32_t data[2];
  uint8_t  type;
} ControllerEvent_t;
google::protobuf::Message* NucleoOutPropulsionControllerEventCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  if ((size_t)data_size != sizeof(ControllerEvent_t))
  {
    printf ("NUCLEO TRANSCODER ERROR : message_id=PropulsionControllerEvent , incorrect data size (%d)\n", data_size);
    return &pb_msg;
  }

  ControllerEvent_t* my_event = (ControllerEvent_t*) data_from_nucleo;

  pb_msg.mutable_pose()->mutable_position()->set_x     (my_event->pose_x);
  pb_msg.mutable_pose()->mutable_position()->set_y     (my_event->pose_y);
  pb_msg.mutable_pose()->set_yaw                       (my_event->pose_yaw);
  pb_msg.mutable_pose()->set_speed                     (my_event->pose_speed);
  pb_msg.mutable_pose()->set_yaw_rate                  (my_event->pose_yaw_rate);
  pb_msg.mutable_pose()->set_acceleration              (my_event->pose_acceleration);
  pb_msg.mutable_pose()->set_angular_acceleration      (my_event->pose_angular_acceleration);
  pb_msg.set_parameter                                 (my_event->parameter);
  pb_msg.set_data1                                     (my_event->data[0]);
  pb_msg.set_data2                                     (my_event->data[1]);
  pb_msg.set_type                                      (my_event->type);

  return &pb_msg;
}

NUCLEO_IN_CODEC( PropulsionPoseTransform , goldo::nucleo::propulsion::CmdTransformPose , "propulsion/pose/transform" , CommMessageType::PropulsionTransformPose );
std::string& NucleoInPropulsionPoseTransformCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdTransformPose* nucleo_in_propulsion_pose_transform_msg = (goldo::nucleo::propulsion::CmdTransformPose*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_pose_transform_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float translation_x = (float) nucleo_in_propulsion_pose_transform_msg->translation().x();
  nucleo_in_buf.append((const char *)&translation_x, sizeof(translation_x));

  float translation_y = (float) nucleo_in_propulsion_pose_transform_msg->translation().y();
  nucleo_in_buf.append((const char *)&translation_y, sizeof(translation_y));

  float rotation = (float) nucleo_in_propulsion_pose_transform_msg->rotation();
  nucleo_in_buf.append((const char *)&rotation, sizeof(rotation));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionSetEventSensorsMask , goldo::nucleo::propulsion::CmdSetEventSensorsMask , "propulsion/set_event_sensors_mask" , CommMessageType::PropulsionSetEventSensorsMask );
std::string& NucleoInPropulsionSetEventSensorsMaskCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdSetEventSensorsMask* nucleo_in_propulsion_set_event_sensors_mask_msg =
    (goldo::nucleo::propulsion::CmdSetEventSensorsMask*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_set_event_sensors_mask_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint32_t mask_rising = (uint32_t) nucleo_in_propulsion_set_event_sensors_mask_msg->mask_rising();
  nucleo_in_buf.append((const char *)&mask_rising, sizeof(mask_rising));

  uint32_t mask_falling = (uint32_t) nucleo_in_propulsion_set_event_sensors_mask_msg->mask_falling();
  nucleo_in_buf.append((const char *)&mask_falling, sizeof(mask_falling));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionAccelerationLimitsSet , goldo::nucleo::propulsion::CmdSetAccelerationLimits , "propulsion/acceleration_limits/set" , CommMessageType::PropulsionSetAccelerationLimits );
std::string& NucleoInPropulsionAccelerationLimitsSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::CmdSetAccelerationLimits* nucleo_in_propulsion_acceleration_limits_set_msg =
    (goldo::nucleo::propulsion::CmdSetAccelerationLimits*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_acceleration_limits_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float accel = (float) nucleo_in_propulsion_acceleration_limits_set_msg->accel();
  nucleo_in_buf.append((const char *)&accel, sizeof(accel));

  float deccel = (float) nucleo_in_propulsion_acceleration_limits_set_msg->deccel();
  nucleo_in_buf.append((const char *)&deccel, sizeof(deccel));

  float angular_accel = (float) nucleo_in_propulsion_acceleration_limits_set_msg->angular_accel();
  nucleo_in_buf.append((const char *)&angular_accel, sizeof(angular_accel));

  float angular_deccel = (float) nucleo_in_propulsion_acceleration_limits_set_msg->angular_deccel();
  nucleo_in_buf.append((const char *)&angular_deccel, sizeof(angular_deccel));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionMotorsTorqueLimitsSet , goldo::nucleo::propulsion::SetMotorsTorqueLimits , "propulsion/motors/torque_limits/set" , CommMessageType::PropulsionMotorsTorqueLimitsSet );
std::string& NucleoInPropulsionMotorsTorqueLimitsSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::SetMotorsTorqueLimits* nucleo_in_propulsion_motors_torque_limits_set_msg =
    (goldo::nucleo::propulsion::SetMotorsTorqueLimits*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_motors_torque_limits_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  float left = (float) nucleo_in_propulsion_motors_torque_limits_set_msg->left();
  nucleo_in_buf.append((const char *)&left, sizeof(left));

  float right = (float) nucleo_in_propulsion_motors_torque_limits_set_msg->right();
  nucleo_in_buf.append((const char *)&right, sizeof(right));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( PropulsionCmdTrajectory , goldo::nucleo::propulsion::ExecuteTrajectory , "propulsion/cmd/trajectory" , CommMessageType::PropulsionExecuteTrajectory );
std::string& NucleoInPropulsionCmdTrajectoryCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::propulsion::ExecuteTrajectory* nucleo_in_propulsion_cmd_trajectory_msg =
    (goldo::nucleo::propulsion::ExecuteTrajectory*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_propulsion_cmd_trajectory_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint16_t dummy = 0;
  nucleo_in_buf.append((const char *)&dummy, sizeof(dummy));

  float speed = (float) nucleo_in_propulsion_cmd_trajectory_msg->speed();
  nucleo_in_buf.append((const char *)&speed, sizeof(speed));

  float reposition_distance = (float) nucleo_in_propulsion_cmd_trajectory_msg->reposition_distance();
  nucleo_in_buf.append((const char *)&reposition_distance, sizeof(reposition_distance));

  float reposition_speed = (float) nucleo_in_propulsion_cmd_trajectory_msg->reposition_speed();
  nucleo_in_buf.append((const char *)&reposition_speed, sizeof(reposition_speed));

  for (int i=0; i<nucleo_in_propulsion_cmd_trajectory_msg->points_size(); i++)
  {
    float x = (float) nucleo_in_propulsion_cmd_trajectory_msg->points(i).x();
    nucleo_in_buf.append((const char *)&x, sizeof(x));

    float y = (float) nucleo_in_propulsion_cmd_trajectory_msg->points(i).y();
    nucleo_in_buf.append((const char *)&y, sizeof(y));
  }

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( ServoAck , google::protobuf::UInt32Value , "servo/ack" , CommMessageType::ServoAck );
google::protobuf::Message* NucleoOutServoAckCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned char *ptr = data_from_nucleo;
  const uint16_t *ack_val = (const uint16_t *) ptr;

  pb_msg.set_value(*ack_val);

  return &pb_msg;
}

NUCLEO_IN_CODEC( ServoDisableAll , goldo::nucleo::servos::CmdDisableAll , "servo/disable_all" , CommMessageType::ServoDisableAll );
std::string& NucleoInServoDisableAllCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdDisableAll* nucleo_in_servo_disable_all_msg =
    (goldo::nucleo::servos::CmdDisableAll*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_servo_disable_all_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( ServoEnableSet , goldo::nucleo::servos::CmdSetEnable , "servo/enable/set" , CommMessageType::ServoSetEnable );
std::string& NucleoInServoEnableSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdSetEnable* nucleo_in_servo_enable_set_msg =
    (goldo::nucleo::servos::CmdSetEnable*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_servo_enable_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  for (int i=0; i<nucleo_in_servo_enable_set_msg->enables_size(); i++)
  {
    uint8_t servo_id = (uint8_t) nucleo_in_servo_enable_set_msg->enables(i).servo_id();
    nucleo_in_buf.append((const char *)&servo_id, sizeof(servo_id));

    uint8_t enable = (uint8_t) nucleo_in_servo_enable_set_msg->enables(i).enable();
    nucleo_in_buf.append((const char *)&enable, sizeof(enable));
  }

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( ServoSetMaxTorques , goldo::nucleo::servos::CmdSetMaxTorques , "servo/set_max_torques" , CommMessageType::ServoSetMaxTorques );
std::string& NucleoInServoSetMaxTorquesCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdSetMaxTorques* nucleo_in_servo_set_max_torques_msg =
    (goldo::nucleo::servos::CmdSetMaxTorques*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_servo_set_max_torques_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  for (int i=0; i<nucleo_in_servo_set_max_torques_msg->torques_size(); i++)
  {
    uint8_t servo_id = (uint8_t) nucleo_in_servo_set_max_torques_msg->torques(i).servo_id();
    nucleo_in_buf.append((const char *)&servo_id, sizeof(servo_id));

    uint8_t torque = (uint8_t) nucleo_in_servo_set_max_torques_msg->torques(i).torque();
    nucleo_in_buf.append((const char *)&torque, sizeof(torque));
  }

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( ServoMoveMultiple , goldo::nucleo::servos::CmdMoveMultiple , "servo/move_multiple" , CommMessageType::ServoMoveMultiple );
std::string& NucleoInServoMoveMultipleCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdMoveMultiple* nucleo_in_servo_move_multiple_msg =
    (goldo::nucleo::servos::CmdMoveMultiple*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_servo_move_multiple_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint16_t speed = (uint16_t) nucleo_in_servo_move_multiple_msg->speed();
  nucleo_in_buf.append((const char *)&speed, sizeof(speed));

  for (int i=0; i<nucleo_in_servo_move_multiple_msg->positions_size(); i++)
  {
    uint8_t servo_id = (uint8_t) nucleo_in_servo_move_multiple_msg->positions(i).servo_id();
    nucleo_in_buf.append((const char *)&servo_id, sizeof(servo_id));

    uint16_t position = (uint16_t) nucleo_in_servo_move_multiple_msg->positions(i).position();
    nucleo_in_buf.append((const char *)&position, sizeof(position));
  }

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( LiftSetEnable , goldo::nucleo::servos::CmdLiftSetEnable , "lift/set_enable" , CommMessageType::LiftSetEnable );
std::string& NucleoInLiftSetEnableCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdLiftSetEnable* nucleo_in_lift_set_enable_msg =
    (goldo::nucleo::servos::CmdLiftSetEnable*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_lift_set_enable_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint8_t lift_id = (uint8_t) nucleo_in_lift_set_enable_msg->lift_id();
  nucleo_in_buf.append((const char *)&lift_id, sizeof(lift_id));

  uint8_t enable = (uint8_t) nucleo_in_lift_set_enable_msg->enable();
  nucleo_in_buf.append((const char *)&enable, sizeof(enable));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( LiftDoHoming , goldo::nucleo::servos::CmdLiftDoHoming , "lift/do_homing" , CommMessageType::LiftDoHoming );
std::string& NucleoInLiftDoHomingCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdLiftDoHoming* nucleo_in_lift_do_homing_msg =
    (goldo::nucleo::servos::CmdLiftDoHoming*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_lift_do_homing_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint8_t lift_id = (uint8_t) nucleo_in_lift_do_homing_msg->lift_id();
  nucleo_in_buf.append((const char *)&lift_id, sizeof(lift_id));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( LiftHomingDone , google::protobuf::UInt32Value , "lift/homing_done" , CommMessageType::LiftHomingDone );
google::protobuf::Message* NucleoOutLiftHomingDoneCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned char *ptr = data_from_nucleo;
  const uint8_t *lift_id = (const uint8_t *) ptr;

  pb_msg.set_value(*lift_id);

  return &pb_msg;
}

NUCLEO_IN_CODEC( LiftCmdRaw , goldo::nucleo::servos::CmdLiftsRaw , "lift/cmd_raw" , CommMessageType::LiftsCmdRaw );
std::string& NucleoInLiftCmdRawCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::servos::CmdLiftsRaw* nucleo_in_lift_cmd_raw_msg =
    (goldo::nucleo::servos::CmdLiftsRaw*) &pb_msg;

  uint16_t sequence_number = (uint16_t) nucleo_in_lift_cmd_raw_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  int32_t lift1_target = (int32_t) nucleo_in_lift_cmd_raw_msg->lift1_target();
  nucleo_in_buf.append((const char *)&lift1_target, sizeof(lift1_target));

  int16_t lift1_bltrig = (int16_t) nucleo_in_lift_cmd_raw_msg->lift1_bltrig();
  nucleo_in_buf.append((const char *)&lift1_bltrig, sizeof(lift1_bltrig));

  int16_t lift1_speed = (int16_t) nucleo_in_lift_cmd_raw_msg->lift1_speed();
  nucleo_in_buf.append((const char *)&lift1_speed, sizeof(lift1_speed));

  int32_t lift2_target = (int32_t) nucleo_in_lift_cmd_raw_msg->lift2_target();
  nucleo_in_buf.append((const char *)&lift2_target, sizeof(lift2_target));

  int16_t lift2_bltrig = (int16_t) nucleo_in_lift_cmd_raw_msg->lift2_bltrig();
  nucleo_in_buf.append((const char *)&lift2_bltrig, sizeof(lift2_bltrig));

  int16_t lift2_speed = (int16_t) nucleo_in_lift_cmd_raw_msg->lift2_speed();
  nucleo_in_buf.append((const char *)&lift2_speed, sizeof(lift2_speed));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( ServoStatusStates , goldo::nucleo::servos::ServoStates , "servo/status/states" , CommMessageType::ServoState );
typedef struct xServoState {
  uint16_t position;
  uint16_t measured_position;
  int16_t  measured_load;
} ServoState_t;
google::protobuf::Message* NucleoOutServoStatusStatesCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  static const int MAX_SERVOS=64;

  int n_servos = (data_size-6)/6;
  if (n_servos>MAX_SERVOS) n_servos = MAX_SERVOS;

  pb_msg.Clear();

  const unsigned char *ptr = data_from_nucleo;
  const uint32_t *timestamp = (const uint32_t *) ptr;
  ptr += 4;
  pb_msg.set_timestamp(*timestamp);

  /* reserved buf? */
  ptr += 2;

  for (int i=0; i<n_servos; i++)
  {
    goldo::nucleo::servos::ServoState *new_servo = pb_msg.add_servos();
    const ServoState_t *servo_data = (const ServoState_t *) ptr;
    ptr += sizeof(ServoState_t);
    new_servo->set_position(servo_data->position);
    new_servo->set_measured_position(servo_data->measured_position);
    new_servo->set_measured_load(servo_data->measured_load);
  }

  return &pb_msg;
}

NUCLEO_OUT_CODEC( ServoStatusMoving , google::protobuf::UInt32Value , "servo/status/moving" , CommMessageType::ServosMoving );
google::protobuf::Message* NucleoOutServoStatusMovingCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const unsigned char *ptr = data_from_nucleo;
  const uint32_t *moving_mask = (const uint32_t *) ptr;

  pb_msg.set_value(*moving_mask);

  return &pb_msg;
}

NUCLEO_IN_CODEC( GpioSet , goldo::nucleo::gpio::CmdGpioSet , "gpio/set" , CommMessageType::DbgGpioSet );
std::string& NucleoInGpioSetCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  goldo::nucleo::gpio::CmdGpioSet* gpio_set_msg = (goldo::nucleo::gpio::CmdGpioSet*) &pb_msg;

  uint16_t sequence_number = gpio_set_msg->sequence_number();
  nucleo_in_buf.append((const char *)&sequence_number, sizeof(sequence_number));

  uint8_t gpio_id = gpio_set_msg->gpio_id();
  nucleo_in_buf.append((const char *)&gpio_id, sizeof(gpio_id));

  uint8_t value = gpio_set_msg->value();
  nucleo_in_buf.append((const char *)&value, sizeof(value));

  return nucleo_in_buf;
}

NUCLEO_IN_CODEC( DbgGoldo , google::protobuf::UInt32Value , "dbg_goldo" , CommMessageType::DbgGoldo );
std::string& NucleoInDbgGoldoCodec::transcode_and_serialize_nucleo_msg ()
{
  nucleo_in_buf.clear();

  google::protobuf::UInt32Value* dbg_goldo_msg = (google::protobuf::UInt32Value*) &pb_msg;
  uint32_t val = dbg_goldo_msg->value();

  nucleo_in_buf.append((const char *)&val, sizeof(val));

  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( DbgGoldo , google::protobuf::UInt32Value , "dbg_goldo" , CommMessageType::DbgGoldo );
google::protobuf::Message* NucleoOutDbgGoldoCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  const uint32_t *my_val = (const uint32_t *)data_from_nucleo;

  pb_msg.set_value(*my_val);

  return &pb_msg;
}

NUCLEO_IN_CODEC( GetNucleoFirmwareVersion , google::protobuf::Empty , "get_nucleo_firmware_version" , CommMessageType::GetNucleoFirmwareVersion );
std::string& NucleoInGetNucleoFirmwareVersionCodec::transcode_and_serialize_nucleo_msg ()
{
  return nucleo_in_buf;
}

NUCLEO_OUT_CODEC( GetNucleoFirmwareVersion , google::protobuf::StringValue , "get_nucleo_firmware_version" , CommMessageType::GetNucleoFirmwareVersion );
google::protobuf::Message* NucleoOutGetNucleoFirmwareVersionCodec::transcode (const unsigned char* data_from_nucleo, int data_size)
{
  pb_msg.Clear();

  pb_msg.set_value((const char*) data_from_nucleo, data_size);

  return &pb_msg;
}

