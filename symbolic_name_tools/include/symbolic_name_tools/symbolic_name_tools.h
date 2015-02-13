#ifndef SYMBOLIC_NAME_TOOLS_H
#define SYMBOLIC_NAME_TOOLS_H

#include <string>
#include <utility>

/// Convenience functions for dealing with names of symbolic objects
/**
 * These follow/assume the convention that an object instance of a certain type
 * will have a name like type_id.
 */

namespace symbolic_name_tools
{

std::string create_name(const std::string & type, unsigned int id);


std::pair<std::string, unsigned int> split_name(const std::string & name, bool & ok);

std::pair<std::string, unsigned int> split_name(const std::string & name);

}

#endif

