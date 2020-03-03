// Protocol Buffers - dji's data interchange format
// Copyright 2008 dji Inc.  All rights reserved.
// https://developers.dji.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of dji Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef DJI_PROTOBUF_ANY_H__
#define DJI_PROTOBUF_ANY_H__

#include <string>

#include <dji/protobuf/stubs/common.h>
#include <dji/protobuf/descriptor.h>
#include <dji/protobuf/message.h>
#include <dji/protobuf/arenastring.h>

namespace dji {
namespace protobuf {
namespace internal {

// Helper class used to implement dji::protobuf::Any.
class LIBPROTOBUF_EXPORT AnyMetadata {
  typedef ArenaStringPtr UrlType;
  typedef ArenaStringPtr ValueType;
 public:
  // AnyMetadata does not take ownership of "type_url" and "value".
  AnyMetadata(UrlType* type_url, ValueType* value);

  // Packs a message using the default type URL prefix: "type.djiapis.com".
  // The resulted type URL will be "type.djiapis.com/<message_full_name>".
  void PackFrom(const Message& message);
  // Packs a message using the given type URL prefix. The type URL will be
  // constructed by concatenating the message type's full name to the prefix
  // with an optional "/" separator if the prefix doesn't already end up "/".
  // For example, both PackFrom(message, "type.djiapis.com") and
  // PackFrom(message, "type.djiapis.com/") yield the same result type
  // URL: "type.djiapis.com/<message_full_name>".
  void PackFrom(const Message& message, const string& type_url_prefix);

  // Unpacks the payload into the given message. Returns false if the message's
  // type doesn't match the type specified in the type URL (i.e., the full
  // name after the last "/" of the type URL doesn't match the message's actual
  // full name) or parsing the payload has failed.
  bool UnpackTo(Message* message) const;

  // Checks whether the type specified in the type URL matches the given type.
  // A type is consdiered matching if its full name matches the full name after
  // the last "/" in the type URL.
  template<typename T>
  bool Is() const {
    return InternalIs(T::default_instance().GetDescriptor());
  }

 private:
  bool InternalIs(const Descriptor* message) const;

  UrlType* type_url_;
  ValueType* value_;

  DJI_DISALLOW_EVIL_CONSTRUCTORS(AnyMetadata);
};

extern const char kAnyFullTypeName[];          // "dji.protobuf.Any".
extern const char kTypedjiApisComPrefix[];  // "type.djiapis.com/".
extern const char kTypedjiProdComPrefix[];  // "type.djiprod.com/".

// Get the proto type name from Any::type_url value. For example, passing
// "type.djiapis.com/rpc.QueryOrigin" will return "rpc.QueryOrigin" in
// *full_type_name. Returns false if the type_url does not have a "/"
// in the type url separating the full type name.
bool ParseAnyTypeUrl(const string& type_url, string* full_type_name);

// See if message is of type dji.protobuf.Any, if so, return the descriptors
// for "type_url" and "value" fields.
bool GetAnyFieldDescriptors(const Message& message,
                            const FieldDescriptor** type_url_field,
                            const FieldDescriptor** value_field);

}  // namespace internal
}  // namespace protobuf

}  // namespace dji
#endif  // DJI_PROTOBUF_ANY_H__
