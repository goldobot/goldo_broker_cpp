// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: goldo/nucleo/gpio.proto

#include "goldo/nucleo/gpio.pb.h"

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
namespace gpio {
class CmdGpioSetDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<CmdGpioSet>
      _instance;
} _CmdGpioSet_default_instance_;
}  // namespace gpio
}  // namespace nucleo
}  // namespace goldo
namespace protobuf_goldo_2fnucleo_2fgpio_2eproto {
static void InitDefaultsCmdGpioSet() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::goldo::nucleo::gpio::_CmdGpioSet_default_instance_;
    new (ptr) ::goldo::nucleo::gpio::CmdGpioSet();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::goldo::nucleo::gpio::CmdGpioSet::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_CmdGpioSet =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsCmdGpioSet}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_CmdGpioSet.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::gpio::CmdGpioSet, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::gpio::CmdGpioSet, sequence_number_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::gpio::CmdGpioSet, gpio_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::goldo::nucleo::gpio::CmdGpioSet, value_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::goldo::nucleo::gpio::CmdGpioSet)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::goldo::nucleo::gpio::_CmdGpioSet_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "goldo/nucleo/gpio.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\027goldo/nucleo/gpio.proto\022\021goldo.nucleo."
      "gpio\032\027goldo/pb2_options.proto\"W\n\nCmdGpio"
      "Set\022\035\n\017sequence_number\030\001 \001(\005B\004\200\265\030\005\022\025\n\007gp"
      "io_id\030\002 \001(\005B\004\200\265\030\003\022\023\n\005value\030\003 \001(\010B\004\200\265\030\003b\006"
      "proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 166);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "goldo/nucleo/gpio.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_goldo_2fnucleo_2fgpio_2eproto
namespace goldo {
namespace nucleo {
namespace gpio {

// ===================================================================

void CmdGpioSet::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CmdGpioSet::kSequenceNumberFieldNumber;
const int CmdGpioSet::kGpioIdFieldNumber;
const int CmdGpioSet::kValueFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CmdGpioSet::CmdGpioSet()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_goldo_2fnucleo_2fgpio_2eproto::scc_info_CmdGpioSet.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:goldo.nucleo.gpio.CmdGpioSet)
}
CmdGpioSet::CmdGpioSet(const CmdGpioSet& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&sequence_number_, &from.sequence_number_,
    static_cast<size_t>(reinterpret_cast<char*>(&value_) -
    reinterpret_cast<char*>(&sequence_number_)) + sizeof(value_));
  // @@protoc_insertion_point(copy_constructor:goldo.nucleo.gpio.CmdGpioSet)
}

void CmdGpioSet::SharedCtor() {
  ::memset(&sequence_number_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&value_) -
      reinterpret_cast<char*>(&sequence_number_)) + sizeof(value_));
}

CmdGpioSet::~CmdGpioSet() {
  // @@protoc_insertion_point(destructor:goldo.nucleo.gpio.CmdGpioSet)
  SharedDtor();
}

void CmdGpioSet::SharedDtor() {
}

void CmdGpioSet::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* CmdGpioSet::descriptor() {
  ::protobuf_goldo_2fnucleo_2fgpio_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_goldo_2fnucleo_2fgpio_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const CmdGpioSet& CmdGpioSet::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_goldo_2fnucleo_2fgpio_2eproto::scc_info_CmdGpioSet.base);
  return *internal_default_instance();
}


void CmdGpioSet::Clear() {
// @@protoc_insertion_point(message_clear_start:goldo.nucleo.gpio.CmdGpioSet)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&sequence_number_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&value_) -
      reinterpret_cast<char*>(&sequence_number_)) + sizeof(value_));
  _internal_metadata_.Clear();
}

bool CmdGpioSet::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:goldo.nucleo.gpio.CmdGpioSet)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // int32 sequence_number = 1 [(.goldo.pb2_options.cpp_type) = UINT16];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &sequence_number_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 gpio_id = 2 [(.goldo.pb2_options.cpp_type) = UINT8];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &gpio_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bool value = 3 [(.goldo.pb2_options.cpp_type) = UINT8];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &value_)));
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
  // @@protoc_insertion_point(parse_success:goldo.nucleo.gpio.CmdGpioSet)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:goldo.nucleo.gpio.CmdGpioSet)
  return false;
#undef DO_
}

void CmdGpioSet::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:goldo.nucleo.gpio.CmdGpioSet)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 sequence_number = 1 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->sequence_number() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(1, this->sequence_number(), output);
  }

  // int32 gpio_id = 2 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->gpio_id() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->gpio_id(), output);
  }

  // bool value = 3 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->value() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(3, this->value(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:goldo.nucleo.gpio.CmdGpioSet)
}

::google::protobuf::uint8* CmdGpioSet::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:goldo.nucleo.gpio.CmdGpioSet)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 sequence_number = 1 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->sequence_number() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(1, this->sequence_number(), target);
  }

  // int32 gpio_id = 2 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->gpio_id() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->gpio_id(), target);
  }

  // bool value = 3 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->value() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(3, this->value(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:goldo.nucleo.gpio.CmdGpioSet)
  return target;
}

size_t CmdGpioSet::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:goldo.nucleo.gpio.CmdGpioSet)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // int32 sequence_number = 1 [(.goldo.pb2_options.cpp_type) = UINT16];
  if (this->sequence_number() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->sequence_number());
  }

  // int32 gpio_id = 2 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->gpio_id() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->gpio_id());
  }

  // bool value = 3 [(.goldo.pb2_options.cpp_type) = UINT8];
  if (this->value() != 0) {
    total_size += 1 + 1;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CmdGpioSet::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:goldo.nucleo.gpio.CmdGpioSet)
  GOOGLE_DCHECK_NE(&from, this);
  const CmdGpioSet* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const CmdGpioSet>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:goldo.nucleo.gpio.CmdGpioSet)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:goldo.nucleo.gpio.CmdGpioSet)
    MergeFrom(*source);
  }
}

void CmdGpioSet::MergeFrom(const CmdGpioSet& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:goldo.nucleo.gpio.CmdGpioSet)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.sequence_number() != 0) {
    set_sequence_number(from.sequence_number());
  }
  if (from.gpio_id() != 0) {
    set_gpio_id(from.gpio_id());
  }
  if (from.value() != 0) {
    set_value(from.value());
  }
}

void CmdGpioSet::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:goldo.nucleo.gpio.CmdGpioSet)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CmdGpioSet::CopyFrom(const CmdGpioSet& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:goldo.nucleo.gpio.CmdGpioSet)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CmdGpioSet::IsInitialized() const {
  return true;
}

void CmdGpioSet::Swap(CmdGpioSet* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CmdGpioSet::InternalSwap(CmdGpioSet* other) {
  using std::swap;
  swap(sequence_number_, other->sequence_number_);
  swap(gpio_id_, other->gpio_id_);
  swap(value_, other->value_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata CmdGpioSet::GetMetadata() const {
  protobuf_goldo_2fnucleo_2fgpio_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_goldo_2fnucleo_2fgpio_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace gpio
}  // namespace nucleo
}  // namespace goldo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::goldo::nucleo::gpio::CmdGpioSet* Arena::CreateMaybeMessage< ::goldo::nucleo::gpio::CmdGpioSet >(Arena* arena) {
  return Arena::CreateInternal< ::goldo::nucleo::gpio::CmdGpioSet >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
