// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: controlCommand.proto

#ifndef PROTOBUF_controlCommand_2eproto__INCLUDED
#define PROTOBUF_controlCommand_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace stoupentoPlugin_msgs {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_controlCommand_2eproto();
void protobuf_AssignDesc_controlCommand_2eproto();
void protobuf_ShutdownFile_controlCommand_2eproto();

class ControlCommand;

// ===================================================================

class ControlCommand : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:stoupentoPlugin_msgs.msgs.ControlCommand) */ {
 public:
  ControlCommand();
  virtual ~ControlCommand();

  ControlCommand(const ControlCommand& from);

  inline ControlCommand& operator=(const ControlCommand& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ControlCommand& default_instance();

  void Swap(ControlCommand* other);

  // implements Message ----------------------------------------------

  inline ControlCommand* New() const { return New(NULL); }

  ControlCommand* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ControlCommand& from);
  void MergeFrom(const ControlCommand& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ControlCommand* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double u0l = 1;
  bool has_u0l() const;
  void clear_u0l();
  static const int kU0LFieldNumber = 1;
  double u0l() const;
  void set_u0l(double value);

  // required double u0r = 2;
  bool has_u0r() const;
  void clear_u0r();
  static const int kU0RFieldNumber = 2;
  double u0r() const;
  void set_u0r(double value);

  // @@protoc_insertion_point(class_scope:stoupentoPlugin_msgs.msgs.ControlCommand)
 private:
  inline void set_has_u0l();
  inline void clear_has_u0l();
  inline void set_has_u0r();
  inline void clear_has_u0r();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double u0l_;
  double u0r_;
  friend void  protobuf_AddDesc_controlCommand_2eproto();
  friend void protobuf_AssignDesc_controlCommand_2eproto();
  friend void protobuf_ShutdownFile_controlCommand_2eproto();

  void InitAsDefaultInstance();
  static ControlCommand* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// ControlCommand

// required double u0l = 1;
inline bool ControlCommand::has_u0l() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ControlCommand::set_has_u0l() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ControlCommand::clear_has_u0l() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ControlCommand::clear_u0l() {
  u0l_ = 0;
  clear_has_u0l();
}
inline double ControlCommand::u0l() const {
  // @@protoc_insertion_point(field_get:stoupentoPlugin_msgs.msgs.ControlCommand.u0l)
  return u0l_;
}
inline void ControlCommand::set_u0l(double value) {
  set_has_u0l();
  u0l_ = value;
  // @@protoc_insertion_point(field_set:stoupentoPlugin_msgs.msgs.ControlCommand.u0l)
}

// required double u0r = 2;
inline bool ControlCommand::has_u0r() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ControlCommand::set_has_u0r() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ControlCommand::clear_has_u0r() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ControlCommand::clear_u0r() {
  u0r_ = 0;
  clear_has_u0r();
}
inline double ControlCommand::u0r() const {
  // @@protoc_insertion_point(field_get:stoupentoPlugin_msgs.msgs.ControlCommand.u0r)
  return u0r_;
}
inline void ControlCommand::set_u0r(double value) {
  set_has_u0r();
  u0r_ = value;
  // @@protoc_insertion_point(field_set:stoupentoPlugin_msgs.msgs.ControlCommand.u0r)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace stoupentoPlugin_msgs

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_controlCommand_2eproto__INCLUDED