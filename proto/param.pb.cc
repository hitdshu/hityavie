// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: param.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "param.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace hityavie {

namespace {

const ::google::protobuf::Descriptor* ImuNoiseParameter_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  ImuNoiseParameter_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_param_2eproto() {
  protobuf_AddDesc_param_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "param.proto");
  GOOGLE_CHECK(file != NULL);
  ImuNoiseParameter_descriptor_ = file->message_type(0);
  static const int ImuNoiseParameter_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, acc_noise_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, gyr_noise_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, ba_noise_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, bg_noise_),
  };
  ImuNoiseParameter_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      ImuNoiseParameter_descriptor_,
      ImuNoiseParameter::default_instance_,
      ImuNoiseParameter_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ImuNoiseParameter, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(ImuNoiseParameter));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_param_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    ImuNoiseParameter_descriptor_, &ImuNoiseParameter::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_param_2eproto() {
  delete ImuNoiseParameter::default_instance_;
  delete ImuNoiseParameter_reflection_;
}

void protobuf_AddDesc_param_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\013param.proto\022\010hityavie\"]\n\021ImuNoiseParam"
    "eter\022\021\n\tacc_noise\030\001 \002(\001\022\021\n\tgyr_noise\030\002 \002"
    "(\001\022\020\n\010ba_noise\030\003 \002(\001\022\020\n\010bg_noise\030\004 \002(\001", 118);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "param.proto", &protobuf_RegisterTypes);
  ImuNoiseParameter::default_instance_ = new ImuNoiseParameter();
  ImuNoiseParameter::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_param_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_param_2eproto {
  StaticDescriptorInitializer_param_2eproto() {
    protobuf_AddDesc_param_2eproto();
  }
} static_descriptor_initializer_param_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int ImuNoiseParameter::kAccNoiseFieldNumber;
const int ImuNoiseParameter::kGyrNoiseFieldNumber;
const int ImuNoiseParameter::kBaNoiseFieldNumber;
const int ImuNoiseParameter::kBgNoiseFieldNumber;
#endif  // !_MSC_VER

ImuNoiseParameter::ImuNoiseParameter()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:hityavie.ImuNoiseParameter)
}

void ImuNoiseParameter::InitAsDefaultInstance() {
}

ImuNoiseParameter::ImuNoiseParameter(const ImuNoiseParameter& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:hityavie.ImuNoiseParameter)
}

void ImuNoiseParameter::SharedCtor() {
  _cached_size_ = 0;
  acc_noise_ = 0;
  gyr_noise_ = 0;
  ba_noise_ = 0;
  bg_noise_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

ImuNoiseParameter::~ImuNoiseParameter() {
  // @@protoc_insertion_point(destructor:hityavie.ImuNoiseParameter)
  SharedDtor();
}

void ImuNoiseParameter::SharedDtor() {
  if (this != default_instance_) {
  }
}

void ImuNoiseParameter::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ImuNoiseParameter::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ImuNoiseParameter_descriptor_;
}

const ImuNoiseParameter& ImuNoiseParameter::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_param_2eproto();
  return *default_instance_;
}

ImuNoiseParameter* ImuNoiseParameter::default_instance_ = NULL;

ImuNoiseParameter* ImuNoiseParameter::New() const {
  return new ImuNoiseParameter;
}

void ImuNoiseParameter::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<ImuNoiseParameter*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  ZR_(acc_noise_, bg_noise_);

#undef OFFSET_OF_FIELD_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool ImuNoiseParameter::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:hityavie.ImuNoiseParameter)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double acc_noise = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &acc_noise_)));
          set_has_acc_noise();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_gyr_noise;
        break;
      }

      // required double gyr_noise = 2;
      case 2: {
        if (tag == 17) {
         parse_gyr_noise:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &gyr_noise_)));
          set_has_gyr_noise();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_ba_noise;
        break;
      }

      // required double ba_noise = 3;
      case 3: {
        if (tag == 25) {
         parse_ba_noise:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &ba_noise_)));
          set_has_ba_noise();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(33)) goto parse_bg_noise;
        break;
      }

      // required double bg_noise = 4;
      case 4: {
        if (tag == 33) {
         parse_bg_noise:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &bg_noise_)));
          set_has_bg_noise();
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
  // @@protoc_insertion_point(parse_success:hityavie.ImuNoiseParameter)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:hityavie.ImuNoiseParameter)
  return false;
#undef DO_
}

void ImuNoiseParameter::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:hityavie.ImuNoiseParameter)
  // required double acc_noise = 1;
  if (has_acc_noise()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->acc_noise(), output);
  }

  // required double gyr_noise = 2;
  if (has_gyr_noise()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->gyr_noise(), output);
  }

  // required double ba_noise = 3;
  if (has_ba_noise()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->ba_noise(), output);
  }

  // required double bg_noise = 4;
  if (has_bg_noise()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->bg_noise(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:hityavie.ImuNoiseParameter)
}

::google::protobuf::uint8* ImuNoiseParameter::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:hityavie.ImuNoiseParameter)
  // required double acc_noise = 1;
  if (has_acc_noise()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->acc_noise(), target);
  }

  // required double gyr_noise = 2;
  if (has_gyr_noise()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->gyr_noise(), target);
  }

  // required double ba_noise = 3;
  if (has_ba_noise()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->ba_noise(), target);
  }

  // required double bg_noise = 4;
  if (has_bg_noise()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->bg_noise(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:hityavie.ImuNoiseParameter)
  return target;
}

int ImuNoiseParameter::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required double acc_noise = 1;
    if (has_acc_noise()) {
      total_size += 1 + 8;
    }

    // required double gyr_noise = 2;
    if (has_gyr_noise()) {
      total_size += 1 + 8;
    }

    // required double ba_noise = 3;
    if (has_ba_noise()) {
      total_size += 1 + 8;
    }

    // required double bg_noise = 4;
    if (has_bg_noise()) {
      total_size += 1 + 8;
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void ImuNoiseParameter::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const ImuNoiseParameter* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const ImuNoiseParameter*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void ImuNoiseParameter::MergeFrom(const ImuNoiseParameter& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_acc_noise()) {
      set_acc_noise(from.acc_noise());
    }
    if (from.has_gyr_noise()) {
      set_gyr_noise(from.gyr_noise());
    }
    if (from.has_ba_noise()) {
      set_ba_noise(from.ba_noise());
    }
    if (from.has_bg_noise()) {
      set_bg_noise(from.bg_noise());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void ImuNoiseParameter::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ImuNoiseParameter::CopyFrom(const ImuNoiseParameter& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ImuNoiseParameter::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000f) != 0x0000000f) return false;

  return true;
}

void ImuNoiseParameter::Swap(ImuNoiseParameter* other) {
  if (other != this) {
    std::swap(acc_noise_, other->acc_noise_);
    std::swap(gyr_noise_, other->gyr_noise_);
    std::swap(ba_noise_, other->ba_noise_);
    std::swap(bg_noise_, other->bg_noise_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata ImuNoiseParameter::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = ImuNoiseParameter_descriptor_;
  metadata.reflection = ImuNoiseParameter_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace hityavie

// @@protoc_insertion_point(global_scope)
