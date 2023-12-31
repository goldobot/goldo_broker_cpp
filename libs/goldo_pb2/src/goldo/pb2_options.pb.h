// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: goldo/pb2_options.proto

#ifndef PROTOBUF_INCLUDED_goldo_2fpb2_5foptions_2eproto
#define PROTOBUF_INCLUDED_goldo_2fpb2_5foptions_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/descriptor.pb.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_goldo_2fpb2_5foptions_2eproto 

namespace protobuf_goldo_2fpb2_5foptions_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_goldo_2fpb2_5foptions_2eproto
namespace goldo {
namespace pb2_options {
}  // namespace pb2_options
}  // namespace goldo
namespace goldo {
namespace pb2_options {

enum CppType {
  UNKNOWN = 0,
  BOOL = 1,
  INT8 = 2,
  UINT8 = 3,
  INT16 = 4,
  UINT16 = 5,
  INT32 = 6,
  UINT32 = 7,
  INT64 = 8,
  UINT64 = 9,
  FLOAT = 10,
  DOUBLE = 11,
  VOID = 12,
  CppType_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  CppType_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool CppType_IsValid(int value);
const CppType CppType_MIN = UNKNOWN;
const CppType CppType_MAX = VOID;
const int CppType_ARRAYSIZE = CppType_MAX + 1;

const ::google::protobuf::EnumDescriptor* CppType_descriptor();
inline const ::std::string& CppType_Name(CppType value) {
  return ::google::protobuf::internal::NameOfEnum(
    CppType_descriptor(), value);
}
inline bool CppType_Parse(
    const ::std::string& name, CppType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<CppType>(
    CppType_descriptor(), name, value);
}
// ===================================================================


// ===================================================================

static const int kCppTypeFieldNumber = 50000;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::EnumTypeTraits< ::goldo::pb2_options::CppType, ::goldo::pb2_options::CppType_IsValid>, 14, false >
  cpp_type;
static const int kMaxCountFieldNumber = 50001;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::PrimitiveTypeTraits< ::google::protobuf::int32 >, 5, false >
  max_count;
static const int kFixedCountFieldNumber = 50002;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::PrimitiveTypeTraits< bool >, 8, false >
  fixed_count;
static const int kCountTypeFieldNumber = 50003;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::EnumTypeTraits< ::goldo::pb2_options::CppType, ::goldo::pb2_options::CppType_IsValid>, 14, false >
  count_type;

// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace pb2_options
}  // namespace goldo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::goldo::pb2_options::CppType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::goldo::pb2_options::CppType>() {
  return ::goldo::pb2_options::CppType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_goldo_2fpb2_5foptions_2eproto
