// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: goldo/camera.proto

#ifndef PROTOBUF_INCLUDED_goldo_2fcamera_2eproto
#define PROTOBUF_INCLUDED_goldo_2fcamera_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_goldo_2fcamera_2eproto 

namespace protobuf_goldo_2fcamera_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[4];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_goldo_2fcamera_2eproto
namespace goldo {
namespace camera {
class Detections;
class DetectionsDefaultTypeInternal;
extern DetectionsDefaultTypeInternal _Detections_default_instance_;
class Detections_Detection;
class Detections_DetectionDefaultTypeInternal;
extern Detections_DetectionDefaultTypeInternal _Detections_Detection_default_instance_;
class Detections_Detection_Corner;
class Detections_Detection_CornerDefaultTypeInternal;
extern Detections_Detection_CornerDefaultTypeInternal _Detections_Detection_Corner_default_instance_;
class Image;
class ImageDefaultTypeInternal;
extern ImageDefaultTypeInternal _Image_default_instance_;
}  // namespace camera
}  // namespace goldo
namespace google {
namespace protobuf {
template<> ::goldo::camera::Detections* Arena::CreateMaybeMessage<::goldo::camera::Detections>(Arena*);
template<> ::goldo::camera::Detections_Detection* Arena::CreateMaybeMessage<::goldo::camera::Detections_Detection>(Arena*);
template<> ::goldo::camera::Detections_Detection_Corner* Arena::CreateMaybeMessage<::goldo::camera::Detections_Detection_Corner>(Arena*);
template<> ::goldo::camera::Image* Arena::CreateMaybeMessage<::goldo::camera::Image>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace goldo {
namespace camera {

enum Image_Encoding {
  Image_Encoding_UNKNOWN = 0,
  Image_Encoding_JPEG = 1,
  Image_Encoding_Image_Encoding_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  Image_Encoding_Image_Encoding_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool Image_Encoding_IsValid(int value);
const Image_Encoding Image_Encoding_Encoding_MIN = Image_Encoding_UNKNOWN;
const Image_Encoding Image_Encoding_Encoding_MAX = Image_Encoding_JPEG;
const int Image_Encoding_Encoding_ARRAYSIZE = Image_Encoding_Encoding_MAX + 1;

const ::google::protobuf::EnumDescriptor* Image_Encoding_descriptor();
inline const ::std::string& Image_Encoding_Name(Image_Encoding value) {
  return ::google::protobuf::internal::NameOfEnum(
    Image_Encoding_descriptor(), value);
}
inline bool Image_Encoding_Parse(
    const ::std::string& name, Image_Encoding* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Image_Encoding>(
    Image_Encoding_descriptor(), name, value);
}
// ===================================================================

class Image : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.camera.Image) */ {
 public:
  Image();
  virtual ~Image();

  Image(const Image& from);

  inline Image& operator=(const Image& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Image(Image&& from) noexcept
    : Image() {
    *this = ::std::move(from);
  }

  inline Image& operator=(Image&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Image& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Image* internal_default_instance() {
    return reinterpret_cast<const Image*>(
               &_Image_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Image* other);
  friend void swap(Image& a, Image& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Image* New() const final {
    return CreateMaybeMessage<Image>(NULL);
  }

  Image* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Image>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Image& from);
  void MergeFrom(const Image& from);
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
  void InternalSwap(Image* other);
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

  typedef Image_Encoding Encoding;
  static const Encoding UNKNOWN =
    Image_Encoding_UNKNOWN;
  static const Encoding JPEG =
    Image_Encoding_JPEG;
  static inline bool Encoding_IsValid(int value) {
    return Image_Encoding_IsValid(value);
  }
  static const Encoding Encoding_MIN =
    Image_Encoding_Encoding_MIN;
  static const Encoding Encoding_MAX =
    Image_Encoding_Encoding_MAX;
  static const int Encoding_ARRAYSIZE =
    Image_Encoding_Encoding_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Encoding_descriptor() {
    return Image_Encoding_descriptor();
  }
  static inline const ::std::string& Encoding_Name(Encoding value) {
    return Image_Encoding_Name(value);
  }
  static inline bool Encoding_Parse(const ::std::string& name,
      Encoding* value) {
    return Image_Encoding_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // bytes data = 4;
  void clear_data();
  static const int kDataFieldNumber = 4;
  const ::std::string& data() const;
  void set_data(const ::std::string& value);
  #if LANG_CXX11
  void set_data(::std::string&& value);
  #endif
  void set_data(const char* value);
  void set_data(const void* value, size_t size);
  ::std::string* mutable_data();
  ::std::string* release_data();
  void set_allocated_data(::std::string* data);

  // uint32 width = 1;
  void clear_width();
  static const int kWidthFieldNumber = 1;
  ::google::protobuf::uint32 width() const;
  void set_width(::google::protobuf::uint32 value);

  // uint32 height = 2;
  void clear_height();
  static const int kHeightFieldNumber = 2;
  ::google::protobuf::uint32 height() const;
  void set_height(::google::protobuf::uint32 value);

  // .goldo.camera.Image.Encoding encoding = 3;
  void clear_encoding();
  static const int kEncodingFieldNumber = 3;
  ::goldo::camera::Image_Encoding encoding() const;
  void set_encoding(::goldo::camera::Image_Encoding value);

  // @@protoc_insertion_point(class_scope:goldo.camera.Image)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr data_;
  ::google::protobuf::uint32 width_;
  ::google::protobuf::uint32 height_;
  int encoding_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fcamera_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Detections_Detection_Corner : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.camera.Detections.Detection.Corner) */ {
 public:
  Detections_Detection_Corner();
  virtual ~Detections_Detection_Corner();

  Detections_Detection_Corner(const Detections_Detection_Corner& from);

  inline Detections_Detection_Corner& operator=(const Detections_Detection_Corner& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Detections_Detection_Corner(Detections_Detection_Corner&& from) noexcept
    : Detections_Detection_Corner() {
    *this = ::std::move(from);
  }

  inline Detections_Detection_Corner& operator=(Detections_Detection_Corner&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Detections_Detection_Corner& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Detections_Detection_Corner* internal_default_instance() {
    return reinterpret_cast<const Detections_Detection_Corner*>(
               &_Detections_Detection_Corner_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Detections_Detection_Corner* other);
  friend void swap(Detections_Detection_Corner& a, Detections_Detection_Corner& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Detections_Detection_Corner* New() const final {
    return CreateMaybeMessage<Detections_Detection_Corner>(NULL);
  }

  Detections_Detection_Corner* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Detections_Detection_Corner>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Detections_Detection_Corner& from);
  void MergeFrom(const Detections_Detection_Corner& from);
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
  void InternalSwap(Detections_Detection_Corner* other);
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

  // int32 x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int32 x() const;
  void set_x(::google::protobuf::int32 value);

  // int32 y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int32 y() const;
  void set_y(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:goldo.camera.Detections.Detection.Corner)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 x_;
  ::google::protobuf::int32 y_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fcamera_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Detections_Detection : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.camera.Detections.Detection) */ {
 public:
  Detections_Detection();
  virtual ~Detections_Detection();

  Detections_Detection(const Detections_Detection& from);

  inline Detections_Detection& operator=(const Detections_Detection& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Detections_Detection(Detections_Detection&& from) noexcept
    : Detections_Detection() {
    *this = ::std::move(from);
  }

  inline Detections_Detection& operator=(Detections_Detection&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Detections_Detection& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Detections_Detection* internal_default_instance() {
    return reinterpret_cast<const Detections_Detection*>(
               &_Detections_Detection_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(Detections_Detection* other);
  friend void swap(Detections_Detection& a, Detections_Detection& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Detections_Detection* New() const final {
    return CreateMaybeMessage<Detections_Detection>(NULL);
  }

  Detections_Detection* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Detections_Detection>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Detections_Detection& from);
  void MergeFrom(const Detections_Detection& from);
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
  void InternalSwap(Detections_Detection* other);
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

  typedef Detections_Detection_Corner Corner;

  // accessors -------------------------------------------------------

  // repeated .goldo.camera.Detections.Detection.Corner corners = 2;
  int corners_size() const;
  void clear_corners();
  static const int kCornersFieldNumber = 2;
  ::goldo::camera::Detections_Detection_Corner* mutable_corners(int index);
  ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection_Corner >*
      mutable_corners();
  const ::goldo::camera::Detections_Detection_Corner& corners(int index) const;
  ::goldo::camera::Detections_Detection_Corner* add_corners();
  const ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection_Corner >&
      corners() const;

  // int32 tag_id = 1;
  void clear_tag_id();
  static const int kTagIdFieldNumber = 1;
  ::google::protobuf::int32 tag_id() const;
  void set_tag_id(::google::protobuf::int32 value);

  // float ux = 3;
  void clear_ux();
  static const int kUxFieldNumber = 3;
  float ux() const;
  void set_ux(float value);

  // float uy = 4;
  void clear_uy();
  static const int kUyFieldNumber = 4;
  float uy() const;
  void set_uy(float value);

  // @@protoc_insertion_point(class_scope:goldo.camera.Detections.Detection)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection_Corner > corners_;
  ::google::protobuf::int32 tag_id_;
  float ux_;
  float uy_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fcamera_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Detections : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:goldo.camera.Detections) */ {
 public:
  Detections();
  virtual ~Detections();

  Detections(const Detections& from);

  inline Detections& operator=(const Detections& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Detections(Detections&& from) noexcept
    : Detections() {
    *this = ::std::move(from);
  }

  inline Detections& operator=(Detections&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Detections& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Detections* internal_default_instance() {
    return reinterpret_cast<const Detections*>(
               &_Detections_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(Detections* other);
  friend void swap(Detections& a, Detections& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Detections* New() const final {
    return CreateMaybeMessage<Detections>(NULL);
  }

  Detections* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Detections>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Detections& from);
  void MergeFrom(const Detections& from);
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
  void InternalSwap(Detections* other);
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

  typedef Detections_Detection Detection;

  // accessors -------------------------------------------------------

  // repeated .goldo.camera.Detections.Detection detections = 1;
  int detections_size() const;
  void clear_detections();
  static const int kDetectionsFieldNumber = 1;
  ::goldo::camera::Detections_Detection* mutable_detections(int index);
  ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection >*
      mutable_detections();
  const ::goldo::camera::Detections_Detection& detections(int index) const;
  ::goldo::camera::Detections_Detection* add_detections();
  const ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection >&
      detections() const;

  // @@protoc_insertion_point(class_scope:goldo.camera.Detections)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection > detections_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_goldo_2fcamera_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Image

// uint32 width = 1;
inline void Image::clear_width() {
  width_ = 0u;
}
inline ::google::protobuf::uint32 Image::width() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Image.width)
  return width_;
}
inline void Image::set_width(::google::protobuf::uint32 value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Image.width)
}

// uint32 height = 2;
inline void Image::clear_height() {
  height_ = 0u;
}
inline ::google::protobuf::uint32 Image::height() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Image.height)
  return height_;
}
inline void Image::set_height(::google::protobuf::uint32 value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Image.height)
}

// .goldo.camera.Image.Encoding encoding = 3;
inline void Image::clear_encoding() {
  encoding_ = 0;
}
inline ::goldo::camera::Image_Encoding Image::encoding() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Image.encoding)
  return static_cast< ::goldo::camera::Image_Encoding >(encoding_);
}
inline void Image::set_encoding(::goldo::camera::Image_Encoding value) {
  
  encoding_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Image.encoding)
}

// bytes data = 4;
inline void Image::clear_data() {
  data_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Image::data() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Image.data)
  return data_.GetNoArena();
}
inline void Image::set_data(const ::std::string& value) {
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:goldo.camera.Image.data)
}
#if LANG_CXX11
inline void Image::set_data(::std::string&& value) {
  
  data_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:goldo.camera.Image.data)
}
#endif
inline void Image::set_data(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:goldo.camera.Image.data)
}
inline void Image::set_data(const void* value, size_t size) {
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:goldo.camera.Image.data)
}
inline ::std::string* Image::mutable_data() {
  
  // @@protoc_insertion_point(field_mutable:goldo.camera.Image.data)
  return data_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Image::release_data() {
  // @@protoc_insertion_point(field_release:goldo.camera.Image.data)
  
  return data_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Image::set_allocated_data(::std::string* data) {
  if (data != NULL) {
    
  } else {
    
  }
  data_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), data);
  // @@protoc_insertion_point(field_set_allocated:goldo.camera.Image.data)
}

// -------------------------------------------------------------------

// Detections_Detection_Corner

// int32 x = 1;
inline void Detections_Detection_Corner::clear_x() {
  x_ = 0;
}
inline ::google::protobuf::int32 Detections_Detection_Corner::x() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.Corner.x)
  return x_;
}
inline void Detections_Detection_Corner::set_x(::google::protobuf::int32 value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Detections.Detection.Corner.x)
}

// int32 y = 2;
inline void Detections_Detection_Corner::clear_y() {
  y_ = 0;
}
inline ::google::protobuf::int32 Detections_Detection_Corner::y() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.Corner.y)
  return y_;
}
inline void Detections_Detection_Corner::set_y(::google::protobuf::int32 value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Detections.Detection.Corner.y)
}

// -------------------------------------------------------------------

// Detections_Detection

// int32 tag_id = 1;
inline void Detections_Detection::clear_tag_id() {
  tag_id_ = 0;
}
inline ::google::protobuf::int32 Detections_Detection::tag_id() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.tag_id)
  return tag_id_;
}
inline void Detections_Detection::set_tag_id(::google::protobuf::int32 value) {
  
  tag_id_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Detections.Detection.tag_id)
}

// repeated .goldo.camera.Detections.Detection.Corner corners = 2;
inline int Detections_Detection::corners_size() const {
  return corners_.size();
}
inline void Detections_Detection::clear_corners() {
  corners_.Clear();
}
inline ::goldo::camera::Detections_Detection_Corner* Detections_Detection::mutable_corners(int index) {
  // @@protoc_insertion_point(field_mutable:goldo.camera.Detections.Detection.corners)
  return corners_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection_Corner >*
Detections_Detection::mutable_corners() {
  // @@protoc_insertion_point(field_mutable_list:goldo.camera.Detections.Detection.corners)
  return &corners_;
}
inline const ::goldo::camera::Detections_Detection_Corner& Detections_Detection::corners(int index) const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.corners)
  return corners_.Get(index);
}
inline ::goldo::camera::Detections_Detection_Corner* Detections_Detection::add_corners() {
  // @@protoc_insertion_point(field_add:goldo.camera.Detections.Detection.corners)
  return corners_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection_Corner >&
Detections_Detection::corners() const {
  // @@protoc_insertion_point(field_list:goldo.camera.Detections.Detection.corners)
  return corners_;
}

// float ux = 3;
inline void Detections_Detection::clear_ux() {
  ux_ = 0;
}
inline float Detections_Detection::ux() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.ux)
  return ux_;
}
inline void Detections_Detection::set_ux(float value) {
  
  ux_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Detections.Detection.ux)
}

// float uy = 4;
inline void Detections_Detection::clear_uy() {
  uy_ = 0;
}
inline float Detections_Detection::uy() const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.Detection.uy)
  return uy_;
}
inline void Detections_Detection::set_uy(float value) {
  
  uy_ = value;
  // @@protoc_insertion_point(field_set:goldo.camera.Detections.Detection.uy)
}

// -------------------------------------------------------------------

// Detections

// repeated .goldo.camera.Detections.Detection detections = 1;
inline int Detections::detections_size() const {
  return detections_.size();
}
inline void Detections::clear_detections() {
  detections_.Clear();
}
inline ::goldo::camera::Detections_Detection* Detections::mutable_detections(int index) {
  // @@protoc_insertion_point(field_mutable:goldo.camera.Detections.detections)
  return detections_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection >*
Detections::mutable_detections() {
  // @@protoc_insertion_point(field_mutable_list:goldo.camera.Detections.detections)
  return &detections_;
}
inline const ::goldo::camera::Detections_Detection& Detections::detections(int index) const {
  // @@protoc_insertion_point(field_get:goldo.camera.Detections.detections)
  return detections_.Get(index);
}
inline ::goldo::camera::Detections_Detection* Detections::add_detections() {
  // @@protoc_insertion_point(field_add:goldo.camera.Detections.detections)
  return detections_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::goldo::camera::Detections_Detection >&
Detections::detections() const {
  // @@protoc_insertion_point(field_list:goldo.camera.Detections.detections)
  return detections_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace camera
}  // namespace goldo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::goldo::camera::Image_Encoding> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::goldo::camera::Image_Encoding>() {
  return ::goldo::camera::Image_Encoding_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_goldo_2fcamera_2eproto