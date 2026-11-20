#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace {

constexpr std::size_t joint_count = 7;
using Joints = std::array<double, joint_count>;
constexpr std::array<std::size_t, 2> moving{1, 3};
constexpr double pi = 3.14159265358979323846;
constexpr double omega = 2.0 * pi * 0.2;
constexpr std::array<double, 2> amplitude{15.0 * pi / 180.0, 15.0 * pi / 180.0};
// FR3 joint limits for joints 2 and 4.
constexpr std::array<double, 2> lower{-1.8361, -3.0770};
constexpr std::array<double, 2> upper{1.8361, -0.1169};
constexpr double limit_margin = 0.03;
constexpr double ramp_seconds = 2.5;
constexpr double motion_seconds = 15.0;
constexpr double settling_seconds = 1.0;
constexpr double hold_error_limit = 0.08;
constexpr double tracking_error_limit = 0.15;
// Hold gains from the libfranka 0.18.2 external joint impedance example.
constexpr Joints hold_kp{600, 0, 600, 0, 250, 150, 50};
constexpr Joints hold_kd{50, 0, 50, 0, 30, 25, 15};
constexpr Joints torque_limit{12, 12, 12, 12, 5, 5, 5};
constexpr double torque_slew_limit = 100.0;  // Nm/s
constexpr std::size_t max_samples = 17000;

volatile std::sig_atomic_t interrupted = 0;

void requestStop(int) {
  interrupted = 1;
}

struct Gains {
  double kp;
  double kd;
};

using ActiveGains = std::array<Gains, moving.size()>;

ActiveGains loadGains(const std::filesystem::path& path) {
  std::ifstream config(path);
  if (!config) {
    throw std::runtime_error("Could not open gain config: " + path.string());
  }
  ActiveGains gains{};
  std::map<std::string, double*> remaining{
      {"joint2_kp", &gains[0].kp}, {"joint2_kd", &gains[0].kd},
      {"joint4_kp", &gains[1].kp}, {"joint4_kd", &gains[1].kd},
  };
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
    double value = 0.0;
    std::string extra;
    if (setting == remaining.end() || !(entry >> value) || !std::isfinite(value) ||
        value <= 0.0 || (entry >> extra)) {
      throw std::runtime_error(path.string() + ":" + std::to_string(number) +
                               ": expected a unique gain key and a positive finite value");
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
  return gains;
}

struct Reference {
  double position;
  double velocity;
  double acceleration;
};

Reference envelope(double time) {
  if (time >= motion_seconds) {
    return {0.0, 0.0, 0.0};
  }
  if (time >= ramp_seconds && time <= motion_seconds - ramp_seconds) {
    return {1.0, 0.0, 0.0};
  }
  const bool rising = time < ramp_seconds;
  const double s = (rising ? time : motion_seconds - time) / ramp_seconds;
  return {
      s * s * s * (10.0 + s * (-15.0 + 6.0 * s)),
      (rising ? 1.0 : -1.0) * 30.0 * s * s * (1.0 - s) * (1.0 - s) / ramp_seconds,
      60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) / (ramp_seconds * ramp_seconds),
  };
}

struct Sample {
  std::uint64_t robot_time_ms;
  double elapsed_seconds;
  Joints position;
  Joints velocity;
  Joints position_ref;
  Joints velocity_ref;
  Joints acceleration_ref;
  Joints torque_measured;
  Joints torque_previous;
  Joints torque_raw;
  Joints torque_command;
  std::array<double, 3> external_force;
};

void writeLog(std::ostream& out, const std::vector<Sample>& samples) {
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << "robot_time_ms,elapsed_s";
  for (const char* name : {"q_rad", "dq_rad_s", "q_ref_rad", "dq_ref_rad_s",
                           "ddq_ref_rad_s2", "tau_measured_Nm", "tau_previous_Nm",
                           "tau_raw_Nm", "tau_command_Nm"}) {
    for (std::size_t j = 1; j <= joint_count; ++j) {
      out << ',' << name << '_' << j;
    }
  }
  out << ",external_fx_N,external_fy_N,external_fz_N,external_force_norm_N\n";
  for (const auto& s : samples) {
    out << s.robot_time_ms << ',' << s.elapsed_seconds;
    for (const auto* group : {&s.position, &s.velocity, &s.position_ref, &s.velocity_ref,
                              &s.acceleration_ref, &s.torque_measured, &s.torque_previous,
                              &s.torque_raw, &s.torque_command}) {
      for (double value : *group) {
        out << ',' << value;
      }
    }
    const auto& [fx, fy, fz] = s.external_force;
    out << ',' << fx << ',' << fy << ',' << fz << ',' << std::hypot(fx, fy, fz) << '\n';
  }
}

std::filesystem::path reserveLogPath() {
  const auto directory = std::filesystem::path(FRANKA_PROJECT_DIR) / "data" / "computed_torque";
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

void checkStart(const Joints& start) {
  for (std::size_t i = 0; i < moving.size(); ++i) {
    const double center = start[moving[i]];
    if (!std::isfinite(center) || center - amplitude[i] <= lower[i] + limit_margin ||
        center + amplitude[i] >= upper[i] - limit_margin) {
      throw std::runtime_error("Joint " + std::to_string(moving[i] + 1) +
                               " is too close to its position limit for the full sine swing");
    }
  }
}

const char* runController(franka::Robot& robot, const franka::Model& model,
                          const ActiveGains& gains, std::vector<Sample>& samples) {
  Joints start{};
  double elapsed = 0.0;
  const char* stop_reason = nullptr;

  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    if (samples.empty()) {
      start = state.q;
      checkStart(start);
    } else {
      elapsed += period.toSec();
    }

    if (interrupted || samples.size() == samples.capacity()) {
      stop_reason = interrupted ? "Interrupted by Ctrl+C" : "Log buffer filled";
      return franka::MotionFinished(franka::Torques(state.tau_J_d));
    }

    Sample sample{};
    sample.robot_time_ms = state.time.toMSec();
    sample.elapsed_seconds = elapsed;
    sample.position = state.q;
    sample.velocity = state.dq;
    sample.position_ref = start;
    sample.torque_measured = state.tau_J;
    sample.torque_previous = state.tau_J_d;
    // Reverse Franka's reaction convention to report the force on the tool.
    const auto& wrench = state.O_F_ext_hat_K;
    sample.external_force = {-wrench[0], -wrench[1], -wrench[2]};

    const Reference w = envelope(elapsed);
    const double sine = std::sin(omega * elapsed);
    const double cosine = std::cos(omega * elapsed);
    Joints acceleration{};
    for (std::size_t i = 0; i < moving.size(); ++i) {
      const auto j = moving[i];
      const double a = amplitude[i];
      sample.position_ref[j] += a * w.position * sine;
      sample.velocity_ref[j] = a * (w.velocity * sine + omega * w.position * cosine);
      sample.acceleration_ref[j] = a * ((w.acceleration - omega * omega * w.position) * sine +
                                        2.0 * omega * w.velocity * cosine);
      acceleration[j] = sample.acceleration_ref[j] +
                        gains[i].kp * (sample.position_ref[j] - state.q[j]) +
                        gains[i].kd * (sample.velocity_ref[j] - state.dq[j]);
    }

    const auto mass = model.mass(state);
    const auto coriolis = model.coriolis(state);
    for (std::size_t j = 0; j < joint_count; ++j) {
      const double error = sample.position_ref[j] - state.q[j];
      const bool active = j == moving[0] || j == moving[1];
      if (active) {
        sample.torque_raw[j] = coriolis[j];
        for (const auto k : moving) {
          sample.torque_raw[j] += mass[k * joint_count + j] * acceleration[k];
        }
      } else {
        sample.torque_raw[j] = hold_kp[j] * error - hold_kd[j] * state.dq[j];
      }

      if (!std::isfinite(sample.torque_raw[j]) || !std::isfinite(state.tau_J_d[j])) {
        throw std::runtime_error("Non-finite torque value");
      }
      if (std::abs(error) > (active ? tracking_error_limit : hold_error_limit)) {
        stop_reason = active ? "Moving joint exceeded tracking error limit"
                             : "Holding joint exceeded position error limit";
      }
      const double bounded = std::clamp(sample.torque_raw[j], -torque_limit[j], torque_limit[j]);
      const double step = torque_slew_limit * std::min(0.001, period.toSec());
      sample.torque_command[j] =
          std::clamp(bounded, state.tau_J_d[j] - step, state.tau_J_d[j] + step);
    }

    if (stop_reason) {
      sample.torque_command = state.tau_J_d;
    }
    samples.push_back(sample);

    const franka::Torques command(sample.torque_command);
    if (stop_reason || elapsed >= motion_seconds + settling_seconds) {
      return franka::MotionFinished(command);
    }
    return command;
  }, true);

  return stop_reason;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0] << " [--ignore-realtime]\n"
              << "Run a 0.2 Hz sine on FR3 joints 2 and 4 (±15 degrees each).\n"
              << "Connects to 172.16.0.2 and saves a numbered CSV in data/computed_torque/.\n"
              << "Reads tracking gains from config/computed_torque.conf.\n"
              << "By default, the controller requires real-time scheduling.\n";
    return 0;
  }
  const bool ignore_realtime = argc > 1 && std::string_view(argv[1]) == "--ignore-realtime";
  if (argc > 1 && (argc != 2 || !ignore_realtime)) {
    std::cerr << "Usage: " << argv[0] << " [--ignore-realtime]\n";
    return 2;
  }

  try {
    const auto config_path = std::filesystem::path(FRANKA_PROJECT_DIR) /
                             "config" / "computed_torque.conf";
    const auto gains = loadGains(config_path);
    for (std::size_t i = 0; i < moving.size(); ++i) {
      std::cout << "Joint " << moving[i] + 1 << " tracking gains: Kp=" << gains[i].kp
                << " Kd=" << gains[i].kd << '\n';
    }
    if (std::signal(SIGINT, requestStop) == SIG_ERR) {
      throw std::runtime_error("Could not register Ctrl+C handler");
    }
    std::vector<Sample> samples;
    samples.reserve(max_samples);

    const auto realtime_config =
        ignore_realtime ? franka::RealtimeConfig::kIgnore : franka::RealtimeConfig::kEnforce;
    franka::Robot robot("172.16.0.2", realtime_config);
    const auto model = robot.loadModel();
    checkStart(robot.readOnce().q);

    const auto log_path = reserveLogPath();
    const std::string path = log_path.string();
    const auto diagnostic_path = log_path.parent_path() / (log_path.stem().string() + ".franka.csv");
    std::ofstream log(path);
    if (!log) {
      throw std::runtime_error("Could not open " + path);
    }

    int result = 0;
    try {
      if (const char* reason = runController(robot, model, gains, samples)) {
        std::cerr << "Controller stopped: " << reason << '\n';
        result = 1;
      }
    } catch (const franka::ControlException& error) {
      std::cerr << "Control failed: " << error.what() << '\n';
      result = 1;
      if (!error.log.empty()) {
        std::ofstream diagnostic(diagnostic_path);
        diagnostic << franka::logToCSV(error.log);
        diagnostic.close();
        if (diagnostic) {
          std::cerr << "Saved libfranka diagnostics to " << diagnostic_path.string() << '\n';
        } else {
          std::cerr << "Could not save libfranka diagnostics to " << diagnostic_path.string() << '\n';
        }
      }
    } catch (const std::exception& error) {
      std::cerr << "Control failed: " << error.what() << '\n';
      result = 1;
    }
    writeLog(log, samples);
    log.close();
    if (!log) {
      throw std::runtime_error("Could not save complete CSV to " + path);
    }
    std::cout << "Saved " << samples.size() << " samples to " << path << '\n';
    return result;
  } catch (const std::exception& error) {
    std::cerr << "Computed torque failed: " << error.what() << '\n';
    return 1;
  }
}
