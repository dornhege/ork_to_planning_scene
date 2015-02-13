#include <sstream>
#include <stdio.h>
#include "symbolic_name_tools/symbolic_name_tools.h"

namespace symbolic_name_tools
{

std::string create_name(const std::string & type, unsigned int id)
{
    std::stringstream ss;
    ss << type << "_" << id;
    return ss.str();
}

std::pair<std::string, unsigned int> split_name(const std::string & name)
{
    bool dontCare;
    return split_name(name, dontCare);
}

std::pair<std::string, unsigned int> split_name(const std::string & name, bool & ok)
{
    ok = true;

    size_t pos = name.find_last_of("_");
    if(pos == std::string::npos) {
        ok = false;
        return std::make_pair(name, 0);
    }
    // extract type
    std::string type = name.substr(0, pos);
    if(type.empty()) {
        ok = false;
    }
    // extract id
    std::string idStr = name.substr(pos + 1);
    std::stringstream ss(idStr);
    unsigned int id;
    if(!(ss >> id)) {
        ok = false;
        id = 0;
    }
    return std::make_pair(type, id);
}

}

