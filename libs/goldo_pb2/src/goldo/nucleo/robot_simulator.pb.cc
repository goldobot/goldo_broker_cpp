// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: goldo/nucleo/robot_simulator.proto

#include "goldo/nucleo/robot_simulator.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace goldo {
namespace nucleo {
namespace robot_simulator {
class RobotSimulatorConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<RobotSimulatorConfig>
      _instance;
} _RobotSimulatorConfig_default_instance_;
}  // namespace robot_simulator
}  // namespace nucleo
}  // namespace goldo
namespace protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto {
static void InitDefaultsRobotSimulatorConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::goldo::nucleo::robot_simulator::_RobotSimulatorConfig_default_instance_;
    new (ptr) ::goldo::nucleo::robot_simulator::RobotSimulatorConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::goldo::nucleo::robot_simulator::RobotSimulatorConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_RobotSimulatorConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsRobotSimulatorConfig}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_RobotSimulatorConfig.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, speed_coeff_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, wheels_spacing_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, encoders_spacing_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, encoders_counts_per_m_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::robot_simulator::RobotSimulatorConfig, encoders_period_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::goldo::nucleo::robot_simulator::RobotSimulatorConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::goldo::nucleo::robot_simulator::_RobotSimulatorConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "goldo/nucleo/robot_simulator.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\"goldo/nucleo/robot_simulator.proto\022\034go"
      "ldo.nucleo.robot_simulator\032\027goldo/pb2_op"
      "tions.proto\"\233\001\n\024RobotSimulatorConfig\022\023\n\013"
      "speed_coeff\030\001 \001(\002\022\026\n\016wheels_spacing\030\002 \001("
      "\002\022\030\n\020encoders_spacing\030\003 \001(\002\022\035\n\025encoders_"
      "counts_per_m\030\004 \001(\002\022\035\n\017encoders_period\030\005 "
      "\001(\rB\004\200\265\030\005b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 257);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "goldo/nucleo/robot_simulator.proto", &protobuf_RegisterTypes);
  ::protobuf_goldo_2fpb2_5foptions_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto
namespace goldo {
namespace nucleo {
namespace robot_simulator {

// ===================================================================

void RobotSimulatorConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RobotSimulatorConfig::kSpeedCoeffFieldNumber;
const int RobotSimulatorConfig::kWheelsSpacingFieldNumber;
const int RobotSimulatorConfig::kEncodersSpacingFieldNumber;
const int RobotSimulatorConfig::kEncodersCountsPerMFieldNumber;
const int RobotSimulatorConfig::kEncodersPeriodFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RobotSimulatorConfig::RobotSimulatorConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::scc_info_RobotSimulatorConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
}
RobotSimulatorConfig::RobotSimulatorConfig(const RobotSimulatorConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&speed_coeff_, &from.speed_coeff_,
    static_cast<size_t>(reinterpret_cast<char*>(&encoders_period_) -
    reinterpret_cast<char*>(&speed_coeff_)) + sizeof(encoders_period_));
  // @@protoc_insertion_point(copy_constructor:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
}

void RobotSimulatorConfig::SharedCtor() {
  ::memset(&speed_coeff_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&encoders_period_) -
      reinterpret_cast<char*>(&speed_coeff_)) + sizeof(encoders_period_));
}

RobotSimulatorConfig::~RobotSimulatorConfig() {
  // @@protoc_insertion_point(destructor:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  SharedDtor();
}

void RobotSimulatorConfig::SharedDtor() {
}

void RobotSimulatorConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* RobotSimulatorConfig::descriptor() {
  ::protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const RobotSimulatorConfig& RobotSimulatorConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::scc_info_RobotSimulatorConfig.base);
  return *internal_default_instance();
}


void RobotSimulatorConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&speed_coeff_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&encoders_period_) -
      reinterpret_cast<char*>(&speed_coeff_)) + sizeof(encoders_period_));
  _internal_metadata_.Clear();
}

bool RobotSimulatorConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float speed_coeff = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &speed_coeff_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float wheels_spacing = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &wheels_spacing_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float encoders_spacing = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &encoders_spacing_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float encoders_counts_per_m = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(37u /* 37 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &encoders_counts_per_m_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // uint32 encoders_period = 5 [(.goldo.pb2_options.cpp_type) = UINT16];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &encoders_period_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  return false;
#undef DO_
}

void RobotSimulatorConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float speed_coeff = 1;
  if (this->speed_coeff() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->speed_coeff(), output);
  }

  // float wheels_spacing = 2;
  if (this->wheels_spacing() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->wheels_spacing(), output);
  }

  // float encoders_spacing = 3;
  if (this->encoders_spacing() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->encoders_spacing(), output);
  }

  // float encoders_counts_per_m = 4;
  if (this->encoders_counts_per_m() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->encoders_counts_per_m(), output);
  }

  // uint32 encoders_period = 5 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->encoders_period() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(5, this->encoders_period(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
}

::google::protobuf::uint8* RobotSimulatorConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float speed_coeff = 1;
  if (this->speed_coeff() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->speed_coeff(), target);
  }

  // float wheels_spacing = 2;
  if (this->wheels_spacing() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->wheels_spacing(), target);
  }

  // float encoders_spacing = 3;
  if (this->encoders_spacing() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->encoders_spacing(), target);
  }

  // float encoders_counts_per_m = 4;
  if (this->encoders_counts_per_m() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->encoders_counts_per_m(), target);
  }

  // uint32 encoders_period = 5 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->encoders_period() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(5, this->encoders_period(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  return target;
}

size_t RobotSimulatorConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // float speed_coeff = 1;
  if (this->speed_coeff() != 0) {
    total_size += 1 + 4;
  }

  // float wheels_spacing = 2;
  if (this->wheels_spacing() != 0) {
    total_size += 1 + 4;
  }

  // float encoders_spacing = 3;
  if (this->encoders_spacing() != 0) {
    total_size += 1 + 4;
  }

  // float encoders_counts_per_m = 4;
  if (this->encoders_counts_per_m() != 0) {
    total_size += 1 + 4;
  }

  // uint32 encoders_period = 5 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->encoders_period() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->encoders_period());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void RobotSimulatorConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const RobotSimulatorConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const RobotSimulatorConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
    MergeFrom(*source);
  }
}

void RobotSimulatorConfig::MergeFrom(const RobotSimulatorConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.speed_coeff() != 0) {
    set_speed_coeff(from.speed_coeff());
  }
  if (from.wheels_spacing() != 0) {
    set_wheels_spacing(from.wheels_spacing());
  }
  if (from.encoders_spacing() != 0) {
    set_encoders_spacing(from.encoders_spacing());
  }
  if (from.encoders_counts_per_m() != 0) {
    set_encoders_counts_per_m(from.encoders_counts_per_m());
  }
  if (from.encoders_period() != 0) {
    set_encoders_period(from.encoders_period());
  }
}

void RobotSimulatorConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RobotSimulatorConfig::CopyFrom(const RobotSimulatorConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:goldo.nucleo.robot_simulator.RobotSimulatorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RobotSimulatorConfig::IsInitialized() const {
  return true;
}

void RobotSimulatorConfig::Swap(RobotSimulatorConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RobotSimulatorConfig::InternalSwap(RobotSimulatorConfig* other) {
  using std::swap;
  swap(speed_coeff_, other->speed_coeff_);
  swap(wheels_spacing_, other->wheels_spacing_);
  swap(encoders_spacing_, other->encoders_spacing_);
  swap(encoders_counts_per_m_, other->encoders_counts_per_m_);
  swap(encoders_period_, other->encoders_period_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata RobotSimulatorConfig::GetMetadata() const {
  protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_goldo_2fnucleo_2frobot_5fsimulator_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace robot_simulator
}  // namespace nucleo
}  // namespace goldo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::goldo::nucleo::robot_simulator::RobotSimulatorConfig* Arena::CreateMaybeMessage< ::goldo::nucleo::robot_simulator::RobotSimulatorConfig >(Arena* arena) {
  return Arena::CreateInternal< ::goldo::nucleo::robot_simulator::RobotSimulatorConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)