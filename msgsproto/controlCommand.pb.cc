// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: controlCommand.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "controlCommand.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace stoupentoPlugin_msgs {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* ControlCommand_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  ControlCommand_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_controlCommand_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_controlCommand_2eproto() {
  protobuf_AddDesc_controlCommand_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "controlCommand.proto");
  GOOGLE_CHECK(file != NULL);
  ControlCommand_descriptor_ = file->message_type(0);
  static const int ControlCommand_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ControlCommand, u0l_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ControlCommand, u0r_),
  };
  ControlCommand_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      ControlCommand_descriptor_,
      ControlCommand::default_instance_,
      ControlCommand_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ControlCommand, _has_bits_[0]),
      -1,
      -1,
      sizeof(ControlCommand),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ControlCommand, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_controlCommand_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      ControlCommand_descriptor_, &ControlCommand::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_controlCommand_2eproto() {
  delete ControlCommand::default_instance_;
  delete ControlCommand_reflection_;
}

void protobuf_AddDesc_controlCommand_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_controlCommand_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\024controlCommand.proto\022\031stoupentoPlugin_"
    "msgs.msgs\"*\n\016ControlCommand\022\013\n\003u0l\030\001 \002(\001"
    "\022\013\n\003u0r\030\002 \002(\001", 93);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "controlCommand.proto", &protobuf_RegisterTypes);
  ControlCommand::default_instance_ = new ControlCommand();
  ControlCommand::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_controlCommand_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_controlCommand_2eproto {
  StaticDescriptorInitializer_controlCommand_2eproto() {
    protobuf_AddDesc_controlCommand_2eproto();
  }
} static_descriptor_initializer_controlCommand_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ControlCommand::kU0LFieldNumber;
const int ControlCommand::kU0RFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ControlCommand::ControlCommand()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:stoupentoPlugin_msgs.msgs.ControlCommand)
}

void ControlCommand::InitAsDefaultInstance() {
}

ControlCommand::ControlCommand(const ControlCommand& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:stoupentoPlugin_msgs.msgs.ControlCommand)
}

void ControlCommand::SharedCtor() {
  _cached_size_ = 0;
  u0l_ = 0;
  u0r_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

ControlCommand::~ControlCommand() {
  // @@protoc_insertion_point(destructor:stoupentoPlugin_msgs.msgs.ControlCommand)
  SharedDtor();
}

void ControlCommand::SharedDtor() {
  if (this != default_instance_) {
  }
}

void ControlCommand::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ControlCommand::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ControlCommand_descriptor_;
}

const ControlCommand& ControlCommand::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_controlCommand_2eproto();
  return *default_instance_;
}

ControlCommand* ControlCommand::default_instance_ = NULL;

ControlCommand* ControlCommand::New(::google::protobuf::Arena* arena) const {
  ControlCommand* n = new ControlCommand;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void ControlCommand::Clear() {
// @@protoc_insertion_point(message_clear_start:stoupentoPlugin_msgs.msgs.ControlCommand)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(ControlCommand, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<ControlCommand*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  ZR_(u0l_, u0r_);

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool ControlCommand::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double u0l = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &u0l_)));
          set_has_u0l();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_u0r;
        break;
      }

      // required double u0r = 2;
      case 2: {
        if (tag == 17) {
         parse_u0r:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &u0r_)));
          set_has_u0r();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:stoupentoPlugin_msgs.msgs.ControlCommand)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:stoupentoPlugin_msgs.msgs.ControlCommand)
  return false;
#undef DO_
}

void ControlCommand::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  // required double u0l = 1;
  if (has_u0l()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->u0l(), output);
  }

  // required double u0r = 2;
  if (has_u0r()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->u0r(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:stoupentoPlugin_msgs.msgs.ControlCommand)
}

::google::protobuf::uint8* ControlCommand::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  // required double u0l = 1;
  if (has_u0l()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->u0l(), target);
  }

  // required double u0r = 2;
  if (has_u0r()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->u0r(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:stoupentoPlugin_msgs.msgs.ControlCommand)
  return target;
}

int ControlCommand::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  int total_size = 0;

  if (has_u0l()) {
    // required double u0l = 1;
    total_size += 1 + 8;
  }

  if (has_u0r()) {
    // required double u0r = 2;
    total_size += 1 + 8;
  }

  return total_size;
}
int ControlCommand::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  int total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required double u0l = 1;
    total_size += 1 + 8;

    // required double u0r = 2;
    total_size += 1 + 8;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void ControlCommand::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const ControlCommand* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const ControlCommand>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:stoupentoPlugin_msgs.msgs.ControlCommand)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:stoupentoPlugin_msgs.msgs.ControlCommand)
    MergeFrom(*source);
  }
}

void ControlCommand::MergeFrom(const ControlCommand& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_u0l()) {
      set_u0l(from.u0l());
    }
    if (from.has_u0r()) {
      set_u0r(from.u0r());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void ControlCommand::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ControlCommand::CopyFrom(const ControlCommand& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:stoupentoPlugin_msgs.msgs.ControlCommand)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCommand::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  return true;
}

void ControlCommand::Swap(ControlCommand* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ControlCommand::InternalSwap(ControlCommand* other) {
  std::swap(u0l_, other->u0l_);
  std::swap(u0r_, other->u0r_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata ControlCommand::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = ControlCommand_descriptor_;
  metadata.reflection = ControlCommand_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// ControlCommand

// required double u0l = 1;
bool ControlCommand::has_u0l() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void ControlCommand::set_has_u0l() {
  _has_bits_[0] |= 0x00000001u;
}
void ControlCommand::clear_has_u0l() {
  _has_bits_[0] &= ~0x00000001u;
}
void ControlCommand::clear_u0l() {
  u0l_ = 0;
  clear_has_u0l();
}
 double ControlCommand::u0l() const {
  // @@protoc_insertion_point(field_get:stoupentoPlugin_msgs.msgs.ControlCommand.u0l)
  return u0l_;
}
 void ControlCommand::set_u0l(double value) {
  set_has_u0l();
  u0l_ = value;
  // @@protoc_insertion_point(field_set:stoupentoPlugin_msgs.msgs.ControlCommand.u0l)
}

// required double u0r = 2;
bool ControlCommand::has_u0r() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void ControlCommand::set_has_u0r() {
  _has_bits_[0] |= 0x00000002u;
}
void ControlCommand::clear_has_u0r() {
  _has_bits_[0] &= ~0x00000002u;
}
void ControlCommand::clear_u0r() {
  u0r_ = 0;
  clear_has_u0r();
}
 double ControlCommand::u0r() const {
  // @@protoc_insertion_point(field_get:stoupentoPlugin_msgs.msgs.ControlCommand.u0r)
  return u0r_;
}
 void ControlCommand::set_u0r(double value) {
  set_has_u0r();
  u0r_ = value;
  // @@protoc_insertion_point(field_set:stoupentoPlugin_msgs.msgs.ControlCommand.u0r)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace stoupentoPlugin_msgs

// @@protoc_insertion_point(global_scope)