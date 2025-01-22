#include "glog_sink.h"
#include <iostream>
#include "ld_log.h"

void LocalGlogSink::send(google::LogSeverity severity,
                         const char* full_filename, const char* base_filename,
                         int line, const google::LogMessageTime& time,
                         const char* message, size_t message_len) {
  if (severity == google::LogSeverity::GLOG_INFO) {
    LdLog::FormatPrint(LdLog::L_INFO, full_filename, line, "%s", message);
  } else if (severity == google::LogSeverity::GLOG_WARNING) {
    LdLog::FormatPrint(LdLog::L_WARN, full_filename, line, "%s", message);
  } else if (severity == google::LogSeverity::GLOG_ERROR) {
    LdLog::FormatPrint(LdLog::L_ERROR, full_filename, line, "%s", message);
  } else {
    LdLog::FormatPrint(LdLog::L_INFO, full_filename, line, "%s", message);
  }
}