// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: goldo/nucleo/fpga.proto

#ifndef PROTOBUF_INCLUDED_goldo_2fnucleo_2ffpga_2eproto
#define PROTOBUF_INCLUDED_goldo_2fnucleo_2ffpga_2eproto

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
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "goldo/pb2_options.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_goldo_2fnucleo_2ffpga_2eproto 

namespace protobuf_goldo_2fnucleo_2ffpga_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[5];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_goldo_2fnucleo_2ffpga_2eproto
namespace goldo {
namespace nucleo {
namespace fpga {
class AdcRead;
class AdcReadDefaultTypeInternal;
extern AdcReadDefaultTypeInternal _AdcRead_default_instance_;
class AdcReadOut;
class AdcReadOutDefaultTypeInternal;
extern AdcReadOutDefaultTypeInternal _AdcReadOut_default_instance_;
class RegRead;
class RegReadDefaultTypeInternal;
extern RegReadDefaultTypeInternal _RegRead_default_instance_;
class RegReadStatus;
class RegReadStatusDefaultTypeInternal;
extern RegReadStatusDefaultTypeInternal _RegReadStatus_default_instance_;
class RegWrite;
class RegWriteDefaultTypeInternal;
extern RegWriteDefaultTypeInternal _RegWrite_default_instance_;
}  // namespace fpga
}  // namespace nucleo
}  // namespace goldo
namespace google {
namespace protobuf {
template<> ::goldo::nucleo::fpga::AdcRead* Arena::CreateMaybeMessage<::goldo::nucleo::fpga::AdcRead>(Arena*);
template<> ::goldo::nucleo::fpga::AdcReadOut* Arena::CreateMaybeMessage<::goldo::nucleo::fpga::AdcReadOut>(Arena*);
template<> ::goldo::nucleo::fpga::RegRead* Arena::CreateMaybeMessage<::goldo::nucleo::fpga::RegRead>(Arena*);
template<> ::goldo::nucleo::fpga::RegReadStatus* Arena::CreateMaybeMessage<::goldo::nucleo::fpga::RegReadStatus>(Arena*);
template<> ::goldo::nucleo::fpga::RegWrite* Arena::CreateMaybeMessage<::goldo::nucleo::fpga::RegWrite>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace goldo {
namespace nucleo {
namespace fpga {

// ===================================================================

class RegRead : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.nucleo.fpga.RegRead) */ {
 public:
  RegRead();
  virtual ~RegRead();

  RegRead(const RegRead& from);

  inline RegRead& operator=(const RegRead& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RegRead(RegRead&& from) noexcept
    : RegRead() {
    *this = ::std::move(from);
  }

  inline RegRead& operator=(RegRead&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RegRead& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RegRead* internal_default_instance() {
    return reinterpret_cast<const RegRead*>(
               &_RegRead_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(RegRead* other);
  friend void swap(RegRead& a, RegRead& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RegRead* New() const final {
    return CreateMaybeMessage<RegRead>(NULL);
  }

  RegRead* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<RegRead>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const RegRead& from);
  void MergeFrom(const RegRead& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(RegRead* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_apb_address();
  static const int kApbAddressFieldNumber = 1;
  ::google::protobuf::uint32 apb_address() const;
  void set_apb_address(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:goldo.nucleo.fpga.RegRead)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 apb_address_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fnucleo_2ffpga_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class RegReadStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.nucleo.fpga.RegReadStatus) */ {
 public:
  RegReadStatus();
  virtual ~RegReadStatus();

  RegReadStatus(const RegReadStatus& from);

  inline RegReadStatus& operator=(const RegReadStatus& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RegReadStatus(RegReadStatus&& from) noexcept
    : RegReadStatus() {
    *this = ::std::move(from);
  }

  inline RegReadStatus& operator=(RegReadStatus&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RegReadStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RegReadStatus* internal_default_instance() {
    return reinterpret_cast<const RegReadStatus*>(
               &_RegReadStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(RegReadStatus* other);
  friend void swap(RegReadStatus& a, RegReadStatus& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RegReadStatus* New() const final {
    return CreateMaybeMessage<RegReadStatus>(NULL);
  }

  RegReadStatus* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<RegReadStatus>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const RegReadStatus& from);
  void MergeFrom(const RegReadStatus& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(RegReadStatus* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_apb_address();
  static const int kApbAddressFieldNumber = 1;
  ::google::protobuf::uint32 apb_address() const;
  void set_apb_address(::google::protobuf::uint32 value);

  // fixed32 apb_value = 2 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_apb_value();
  static const int kApbValueFieldNumber = 2;
  ::google::protobuf::uint32 apb_value() const;
  void set_apb_value(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:goldo.nucleo.fpga.RegReadStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 apb_address_;
  ::google::protobuf::uint32 apb_value_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fnucleo_2ffpga_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class RegWrite : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.nucleo.fpga.RegWrite) */ {
 public:
  RegWrite();
  virtual ~RegWrite();

  RegWrite(const RegWrite& from);

  inline RegWrite& operator=(const RegWrite& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RegWrite(RegWrite&& from) noexcept
    : RegWrite() {
    *this = ::std::move(from);
  }

  inline RegWrite& operator=(RegWrite&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RegWrite& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RegWrite* internal_default_instance() {
    return reinterpret_cast<const RegWrite*>(
               &_RegWrite_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(RegWrite* other);
  friend void swap(RegWrite& a, RegWrite& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RegWrite* New() const final {
    return CreateMaybeMessage<RegWrite>(NULL);
  }

  RegWrite* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<RegWrite>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const RegWrite& from);
  void MergeFrom(const RegWrite& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(RegWrite* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_apb_address();
  static const int kApbAddressFieldNumber = 1;
  ::google::protobuf::uint32 apb_address() const;
  void set_apb_address(::google::protobuf::uint32 value);

  // fixed32 apb_value = 2 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_apb_value();
  static const int kApbValueFieldNumber = 2;
  ::google::protobuf::uint32 apb_value() const;
  void set_apb_value(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:goldo.nucleo.fpga.RegWrite)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 apb_address_;
  ::google::protobuf::uint32 apb_value_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fnucleo_2ffpga_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class AdcRead : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.nucleo.fpga.AdcRead) */ {
 public:
  AdcRead();
  virtual ~AdcRead();

  AdcRead(const AdcRead& from);

  inline AdcRead& operator=(const AdcRead& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  AdcRead(AdcRead&& from) noexcept
    : AdcRead() {
    *this = ::std::move(from);
  }

  inline AdcRead& operator=(AdcRead&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const AdcRead& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AdcRead* internal_default_instance() {
    return reinterpret_cast<const AdcRead*>(
               &_AdcRead_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(AdcRead* other);
  friend void swap(AdcRead& a, AdcRead& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline AdcRead* New() const final {
    return CreateMaybeMessage<AdcRead>(NULL);
  }

  AdcRead* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<AdcRead>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const AdcRead& from);
  void MergeFrom(const AdcRead& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(AdcRead* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 chan = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_chan();
  static const int kChanFieldNumber = 1;
  ::google::protobuf::uint32 chan() const;
  void set_chan(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:goldo.nucleo.fpga.AdcRead)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 chan_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fnucleo_2ffpga_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class AdcReadOut : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.nucleo.fpga.AdcReadOut) */ {
 public:
  AdcReadOut();
  virtual ~AdcReadOut();

  AdcReadOut(const AdcReadOut& from);

  inline AdcReadOut& operator=(const AdcReadOut& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  AdcReadOut(AdcReadOut&& from) noexcept
    : AdcReadOut() {
    *this = ::std::move(from);
  }

  inline AdcReadOut& operator=(AdcReadOut&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const AdcReadOut& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AdcReadOut* internal_default_instance() {
    return reinterpret_cast<const AdcReadOut*>(
               &_AdcReadOut_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    4;

  void Swap(AdcReadOut* other);
  friend void swap(AdcReadOut& a, AdcReadOut& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline AdcReadOut* New() const final {
    return CreateMaybeMessage<AdcReadOut>(NULL);
  }

  AdcReadOut* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<AdcReadOut>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const AdcReadOut& from);
  void MergeFrom(const AdcReadOut& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(AdcReadOut* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 chan = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
  void clear_chan();
  static const int kChanFieldNumber = 1;
  ::google::protobuf::uint32 chan() const;
  void set_chan(::google::protobuf::uint32 value);

  // float chan_val = 2;
  void clear_chan_val();
  static const int kChanValFieldNumber = 2;
  float chan_val() const;
  void set_chan_val(float value);

  // @@protoc_insertion_point(class_scope:goldo.nucleo.fpga.AdcReadOut)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 chan_;
  float chan_val_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fnucleo_2ffpga_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RegRead

// fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void RegRead::clear_apb_address() {
  apb_address_ = 0u;
}
inline ::google::protobuf::uint32 RegRead::apb_address() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.RegRead.apb_address)
  return apb_address_;
}
inline void RegRead::set_apb_address(::google::protobuf::uint32 value) {
  
  apb_address_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.RegRead.apb_address)
}

// -------------------------------------------------------------------

// RegReadStatus

// fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void RegReadStatus::clear_apb_address() {
  apb_address_ = 0u;
}
inline ::google::protobuf::uint32 RegReadStatus::apb_address() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.RegReadStatus.apb_address)
  return apb_address_;
}
inline void RegReadStatus::set_apb_address(::google::protobuf::uint32 value) {
  
  apb_address_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.RegReadStatus.apb_address)
}

// fixed32 apb_value = 2 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void RegReadStatus::clear_apb_value() {
  apb_value_ = 0u;
}
inline ::google::protobuf::uint32 RegReadStatus::apb_value() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.RegReadStatus.apb_value)
  return apb_value_;
}
inline void RegReadStatus::set_apb_value(::google::protobuf::uint32 value) {
  
  apb_value_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.RegReadStatus.apb_value)
}

// -------------------------------------------------------------------

// RegWrite

// fixed32 apb_address = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void RegWrite::clear_apb_address() {
  apb_address_ = 0u;
}
inline ::google::protobuf::uint32 RegWrite::apb_address() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.RegWrite.apb_address)
  return apb_address_;
}
inline void RegWrite::set_apb_address(::google::protobuf::uint32 value) {
  
  apb_address_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.RegWrite.apb_address)
}

// fixed32 apb_value = 2 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void RegWrite::clear_apb_value() {
  apb_value_ = 0u;
}
inline ::google::protobuf::uint32 RegWrite::apb_value() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.RegWrite.apb_value)
  return apb_value_;
}
inline void RegWrite::set_apb_value(::google::protobuf::uint32 value) {
  
  apb_value_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.RegWrite.apb_value)
}

// -------------------------------------------------------------------

// AdcRead

// fixed32 chan = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void AdcRead::clear_chan() {
  chan_ = 0u;
}
inline ::google::protobuf::uint32 AdcRead::chan() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.AdcRead.chan)
  return chan_;
}
inline void AdcRead::set_chan(::google::protobuf::uint32 value) {
  
  chan_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.AdcRead.chan)
}

// -------------------------------------------------------------------

// AdcReadOut

// fixed32 chan = 1 [(.goldo.pb2_options.cpp_type) = UINT32];
inline void AdcReadOut::clear_chan() {
  chan_ = 0u;
}
inline ::google::protobuf::uint32 AdcReadOut::chan() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.AdcReadOut.chan)
  return chan_;
}
inline void AdcReadOut::set_chan(::google::protobuf::uint32 value) {
  
  chan_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.AdcReadOut.chan)
}

// float chan_val = 2;
inline void AdcReadOut::clear_chan_val() {
  chan_val_ = 0;
}
inline float AdcReadOut::chan_val() const {
  // @@protoc_insertion_point(field_get:goldo.nucleo.fpga.AdcReadOut.chan_val)
  return chan_val_;
}
inline void AdcReadOut::set_chan_val(float value) {
  
  chan_val_ = value;
  // @@protoc_insertion_point(field_set:goldo.nucleo.fpga.AdcReadOut.chan_val)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace fpga
}  // namespace nucleo
}  // namespace goldo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_goldo_2fnucleo_2ffpga_2eproto
