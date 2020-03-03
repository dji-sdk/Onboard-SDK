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

#ifndef DJI_PROTOBUF_STUBS_LOGGING_H_
#define DJI_PROTOBUF_STUBS_LOGGING_H_

#include <dji/protobuf/stubs/macros.h>
#include <dji/protobuf/stubs/port.h>

// ===================================================================
// emulates dji3/base/logging.h

namespace dji {
namespace protobuf {

enum LogLevel {
  LOGLEVEL_INFO,     // Informational.  This is never actually used by
                     // libprotobuf.
  LOGLEVEL_WARNING,  // Warns about issues that, although not technically a
                     // problem now, could cause problems in the future.  For
                     // example, a // warning will be printed when parsing a
                     // message that is near the message size limit.
  LOGLEVEL_ERROR,    // An error occurred which should never happen during
                     // normal use.
  LOGLEVEL_FATAL,    // An error occurred from which the library cannot
                     // recover.  This usually indicates a programming error
                     // in the code which calls the library, especially when
                     // compiled in debug mode.

#ifdef NDEBUG
  LOGLEVEL_DFATAL = LOGLEVEL_ERROR
#else
  LOGLEVEL_DFATAL = LOGLEVEL_FATAL
#endif
};

class StringPiece;
namespace util {
class Status;
}
class uint128;
namespace internal {

class LogFinisher;

class LIBPROTOBUF_EXPORT LogMessage {
 public:
  LogMessage(LogLevel level, const char* filename, int line);
  ~LogMessage();

  LogMessage& operator<<(const std::string& value);
  LogMessage& operator<<(const char* value);
  LogMessage& operator<<(char value);
  LogMessage& operator<<(int value);
  LogMessage& operator<<(uint value);
  LogMessage& operator<<(long value);
  LogMessage& operator<<(unsigned long value);
  LogMessage& operator<<(long long value);
  LogMessage& operator<<(unsigned long long value);
  LogMessage& operator<<(double value);
  LogMessage& operator<<(void* value);
  LogMessage& operator<<(const StringPiece& value);
  LogMessage& operator<<(const ::dji::protobuf::util::Status& status);
  LogMessage& operator<<(const uint128& value);

 private:
  friend class LogFinisher;
  void Finish();

  LogLevel level_;
  const char* filename_;
  int line_;
  std::string message_;
};

// Used to make the entire "LOG(BLAH) << etc." expression have a void return
// type and print a newline after each message.
class LIBPROTOBUF_EXPORT LogFinisher {
 public:
  void operator=(LogMessage& other);
};

template<typename T>
bool IsOk(T status) { return status.ok(); }
template<>
inline bool IsOk(bool status) { return status; }

}  // namespace internal

// Undef everything in case we're being mixed with some other dji library
// which already defined them itself.  Presumably all dji libraries will
// support the same syntax for these so it should not be a big deal if they
// end up using our definitions instead.
#undef DJI_LOG
#undef DJI_LOG_IF

#undef DJI_CHECK
#undef DJI_CHECK_OK
#undef DJI_CHECK_EQ
#undef DJI_CHECK_NE
#undef DJI_CHECK_LT
#undef DJI_CHECK_LE
#undef DJI_CHECK_GT
#undef DJI_CHECK_GE
#undef DJI_CHECK_NOTNULL

#undef DJI_DLOG
#undef DJI_DCHECK
#undef DJI_DCHECK_OK
#undef DJI_DCHECK_EQ
#undef DJI_DCHECK_NE
#undef DJI_DCHECK_LT
#undef DJI_DCHECK_LE
#undef DJI_DCHECK_GT
#undef DJI_DCHECK_GE

#define DJI_LOG(LEVEL)                                                 \
  ::dji::protobuf::internal::LogFinisher() =                           \
    ::dji::protobuf::internal::LogMessage(                             \
      ::dji::protobuf::LOGLEVEL_##LEVEL, __FILE__, __LINE__)
#define DJI_LOG_IF(LEVEL, CONDITION) \
  !(CONDITION) ? (void)0 : DJI_LOG(LEVEL)

#define DJI_CHECK(EXPRESSION) \
  DJI_LOG_IF(FATAL, !(EXPRESSION)) << "CHECK failed: " #EXPRESSION ": "
#define DJI_CHECK_OK(A) DJI_CHECK(::dji::protobuf::internal::IsOk(A))
#define DJI_CHECK_EQ(A, B) DJI_CHECK((A) == (B))
#define DJI_CHECK_NE(A, B) DJI_CHECK((A) != (B))
#define DJI_CHECK_LT(A, B) DJI_CHECK((A) <  (B))
#define DJI_CHECK_LE(A, B) DJI_CHECK((A) <= (B))
#define DJI_CHECK_GT(A, B) DJI_CHECK((A) >  (B))
#define DJI_CHECK_GE(A, B) DJI_CHECK((A) >= (B))

namespace internal {
template<typename T>
T* CheckNotNull(const char* /* file */, int /* line */,
                const char* name, T* val) {
  if (val == NULL) {
    DJI_LOG(FATAL) << name;
  }
  return val;
}
}  // namespace internal
#define DJI_CHECK_NOTNULL(A) \
  ::dji::protobuf::internal::CheckNotNull(\
      __FILE__, __LINE__, "'" #A "' must not be NULL", (A))

#ifdef NDEBUG

#define DJI_DLOG(LEVEL) DJI_LOG_IF(LEVEL, false)

#define DJI_DCHECK(EXPRESSION) while(false) DJI_CHECK(EXPRESSION)
#define DJI_DCHECK_OK(E) DJI_DCHECK(::dji::protobuf::internal::IsOk(E))
#define DJI_DCHECK_EQ(A, B) DJI_DCHECK((A) == (B))
#define DJI_DCHECK_NE(A, B) DJI_DCHECK((A) != (B))
#define DJI_DCHECK_LT(A, B) DJI_DCHECK((A) <  (B))
#define DJI_DCHECK_LE(A, B) DJI_DCHECK((A) <= (B))
#define DJI_DCHECK_GT(A, B) DJI_DCHECK((A) >  (B))
#define DJI_DCHECK_GE(A, B) DJI_DCHECK((A) >= (B))

#else  // NDEBUG

#define DJI_DLOG DJI_LOG

#define DJI_DCHECK    DJI_CHECK
#define DJI_DCHECK_OK DJI_CHECK_OK
#define DJI_DCHECK_EQ DJI_CHECK_EQ
#define DJI_DCHECK_NE DJI_CHECK_NE
#define DJI_DCHECK_LT DJI_CHECK_LT
#define DJI_DCHECK_LE DJI_CHECK_LE
#define DJI_DCHECK_GT DJI_CHECK_GT
#define DJI_DCHECK_GE DJI_CHECK_GE

#endif  // !NDEBUG

typedef void LogHandler(LogLevel level, const char* filename, int line,
                        const std::string& message);

// The protobuf library sometimes writes warning and error messages to
// stderr.  These messages are primarily useful for developers, but may
// also help end users figure out a problem.  If you would prefer that
// these messages be sent somewhere other than stderr, call SetLogHandler()
// to set your own handler.  This returns the old handler.  Set the handler
// to NULL to ignore log messages (but see also LogSilencer, below).
//
// Obviously, SetLogHandler is not thread-safe.  You should only call it
// at initialization time, and probably not from library code.  If you
// simply want to suppress log messages temporarily (e.g. because you
// have some code that tends to trigger them frequently and you know
// the warnings are not important to you), use the LogSilencer class
// below.
LIBPROTOBUF_EXPORT LogHandler* SetLogHandler(LogHandler* new_func);

// Create a LogSilencer if you want to temporarily suppress all log
// messages.  As long as any LogSilencer objects exist, non-fatal
// log messages will be discarded (the current LogHandler will *not*
// be called).  Constructing a LogSilencer is thread-safe.  You may
// accidentally suppress log messages occurring in another thread, but
// since messages are generally for debugging purposes only, this isn't
// a big deal.  If you want to intercept log messages, use SetLogHandler().
class LIBPROTOBUF_EXPORT LogSilencer {
 public:
  LogSilencer();
  ~LogSilencer();
};

}  // namespace protobuf
}  // namespace dji

#endif  // DJI_PROTOBUF_STUBS_LOGGING_H_
