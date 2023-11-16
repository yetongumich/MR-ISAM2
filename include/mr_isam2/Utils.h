#pragma once
#include <string>
#include <utility>
#include <vector>

template <typename Map>
bool ContainerEqual(Map const &lhs, Map const &rhs)
{
  return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

std::string Quoted(const std::string &str)
{
  return "\"" + str + "\"";
}

typedef std::pair<std::string, std::string> AttributeType;

/**
 * @brief combine key value pairs into a dict in json format
 * @param[in] items         key value paris
 * @param[in] num_indents   number of indents for each item, -1 for no
 * indent and no newline
 * @return                  a string representing dict in json format
 */
std::string JsonDict(const std::vector<AttributeType> &items,
                     const int num_indents = 0)
{
  std::string indents;
  std::string s;
  if (num_indents < 0)
  {
    indents = "";
    s = "{";
  }
  else
  {
    indents = "\n" + std::string(num_indents, ' ');
    s = std::string(num_indents, ' ') + "{";
  }
  for (auto &item : items)
  {
    s += indents + item.first + ":" + item.second + ",";
  }
  s.pop_back(); // remove the last comma
  s += indents + "}";
  return s;
}

/**
 * @brief combine items into a list in json format
 * @param[in] items         vector of items
 * @param[in] num_indents   number of indents for each item, -1 for no
 * indent and no newline
 * @return                  a string representing list in json format
 */
std::string JsonList(const std::vector<std::string> &items,
                     const int num_indents = 0)
{
  std::string indents;
  std::string s;
  if (num_indents < 0)
  {
    indents = "";
    s = "[";
  }
  else
  {
    indents = "\n" + std::string(num_indents, ' ');
    s = std::string(num_indents, ' ') + "[";
  }
  for (auto &item : items)
  {
    s += indents + item + ",";
  }
  s.pop_back(); // remove the last comma
  s += indents + "]";
  return s;
}