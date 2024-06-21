#ifndef _JARVIS_OPTION_PARSE_
#define _JARVIS_OPTION_PARSE_
#include <string>
namespace slip_detect {
template <typename Option>
void ParseYAMLOption(const std::string &file, Option *option);
}
#endif
