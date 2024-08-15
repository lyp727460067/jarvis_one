#ifndef JARVIS_GLOG_SINK_
#define JARVIS_GLOG_SINK_
#include "glog/logging.h"
//
class LocalGlogSink : public google::LogSink {
 public:
  LocalGlogSink(){}
  void send(google::LogSeverity severity, const char* full_filename,
            const char* base_filename, int line,
            const google::LogMessageTime& time, const char* message,
            size_t message_len);
  ~LocalGlogSink() {}
};

#endif