#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

inline void readGainConfig(const std::filesystem::path& path,
                           std::map<std::string, double*> remaining,
                           const std::set<std::string>& signed_keys = {},
                           std::map<std::string, std::string*> text_keys = {}) {
  std::ifstream config(path);
  if (!config) {
    throw std::runtime_error("Could not open gain config: " + path.string());
  }
  std::string line;
  for (std::size_t number = 1; std::getline(config, line); ++number) {
    const auto comment = line.find('#');
    std::istringstream entry(line.substr(0, comment));
    entry.imbue(std::locale::classic());
    std::string key;
    if (!(entry >> key)) {
      continue;
    }

    const auto setting = remaining.find(key);
    const auto text = text_keys.find(key);
    double value = 0.0;
    std::string extra;
    if (text != text_keys.end()) {
      if (!(entry >> *text->second) || (entry >> extra)) {
        throw std::runtime_error(path.string() + ":" + std::to_string(number) +
                                 ": expected a single value for " + key);
      }
      text_keys.erase(text);
      continue;
    }
    if (setting == remaining.end() || !(entry >> value) || !std::isfinite(value) ||
        (signed_keys.count(key) == 0 && value <= 0.0) || (entry >> extra)) {
      throw std::runtime_error(path.string() + ":" + std::to_string(number) +
                               ": invalid or duplicate setting " + key);
    }
    *setting->second = value;
    remaining.erase(setting);
  }
  if (!config.eof()) {
    throw std::runtime_error("Could not read gain config: " + path.string());
  }
  if (!remaining.empty()) {
    throw std::runtime_error("Missing " + remaining.begin()->first + " in " + path.string());
  }
  if (!text_keys.empty()) {
    throw std::runtime_error("Missing " + text_keys.begin()->first + " in " + path.string());
  }
}
