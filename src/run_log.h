#pragma once

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

inline std::filesystem::path reserveLogPath(const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);

  std::uint64_t last_run = 0;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.path().extension() != ".csv") {
      continue;
    }
    auto stem = entry.path().stem();
    if (stem.extension() == ".franka") {
      stem = stem.stem();
    }
    const auto name = stem.string();
    std::uint64_t run = 0;
    const auto end = name.data() + name.size();
    const auto parsed = std::from_chars(name.data(), end, run);
    if (parsed.ec == std::errc{} && parsed.ptr == end) {
      last_run = std::max(last_run, run);
    }
  }

  while (last_run < std::numeric_limits<std::uint64_t>::max()) {
    std::ostringstream name;
    name.imbue(std::locale::classic());
    name << std::setfill('0') << std::setw(3) << ++last_run << ".csv";
    const auto path = directory / name.str();
    // Reserve the name without overwriting a run started by another process.
    const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd >= 0) {
      close(fd);
      return path;
    }
    const int error = errno;
    if (error != EEXIST) {
      throw std::system_error(error, std::generic_category(), "Could not reserve " + path.string());
    }
  }
  throw std::runtime_error("No run numbers remain");
}
