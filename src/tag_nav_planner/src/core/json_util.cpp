#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <jsoncpp/json/json.h>

#include <tag_nav_planner/core/json_util.h>

namespace tag_nav_planner
{
namespace core
{

Json::Value parseJsonFile(const std::string& path, const std::string& source)
{
  std::ifstream input(path.c_str());
  if (!input)
    throw std::runtime_error("unable to open " + source + ": " + path);
  std::ostringstream contents;
  contents << input.rdbuf();

  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::string errors;
  Json::Value root;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  const std::string text = contents.str();
  if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors))
    throw std::runtime_error("invalid " + source + ": " + errors);
  return root;
}

std::vector<double> numberArray(const Json::Value& value, const std::string& name)
{
  if (!value.isArray())
    throw std::runtime_error(name + " must be an array");
  std::vector<double> output;
  output.reserve(value.size());
  for (Json::ArrayIndex index = 0; index < value.size(); ++index)
  {
    if (!value[index].isNumeric())
      throw std::runtime_error(name + " must contain only numbers");
    output.push_back(value[index].asDouble());
  }
  return output;
}

}  // namespace core
}  // namespace tag_nav_planner
