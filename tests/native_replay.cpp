#include "configuration.h"
#include <fast_livo2/mapper.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

struct Record {
  uint64_t stamp, available, offset;
  uint32_t count, width = 0, height = 0;
};
struct Event { uint64_t time; size_t order, index; char kind; };
int main(int argc, char **argv) {
  if (argc < 3) { std::cerr << "native_replay DATASET OUTPUT [FRONTEND_YAML]\n"; return 2; }
  const std::filesystem::path root(argv[1]), output(argv[2]);
  const auto config = YAML::LoadFile(argc > 3 ? argv[3] : (root / "frontend.yaml").string());
  const auto camera = YAML::LoadFile((root / "camera.yaml").string());
  const auto options = fast_livo2::adapter::readConfig(
      [&](const char *key, auto &value) {
        YAML::Node field; field.reset(config);
        std::istringstream path(key); std::string component;
        while (std::getline(path, component, '.') && field) field.reset(field[component]);
        if (field) value = field.as<std::decay_t<decltype(value)>>();
      }, [&](const char *key, auto &value) {
        if (camera[key]) value = camera[key].as<std::decay_t<decltype(value)>>();
      });
  std::filesystem::create_directories(output);
  std::ofstream trajectory(output / "frontend.csv");
  trajectory << std::setprecision(17);
  std::ofstream twist(output / "twist.csv");
  twist << std::setprecision(17);
  std::vector<Record> lidar, images;
  std::vector<Event> events;
  std::ifstream cloud_file(root / "lidar.bin", std::ios::binary), image_file(root / "camera.bin", std::ios::binary);
  for (auto kind : {'l', 'c'}) {
    auto &file = kind == 'l' ? cloud_file : image_file;
    auto &records = kind == 'l' ? lidar : images;
    Record record{};
    while (file.read(reinterpret_cast<char *>(&record.stamp), 8)) {
      file.read(reinterpret_cast<char *>(&record.available), 8);
      if (kind == 'c') {
        file.read(reinterpret_cast<char *>(&record.width), 4);
        file.read(reinterpret_cast<char *>(&record.height), 4);
      }
      file.read(reinterpret_cast<char *>(&record.count), 4);
      record.offset = file.tellg();
      if (record.stamp >= 4000000000ULL) events.push_back({record.available, events.size(), records.size(), kind});
      records.push_back(record);
      file.seekg(static_cast<uint64_t>(record.count) * (kind == 'l' ? 25 : 1), std::ios::cur);
    }
    file.clear();
  }
  std::ifstream imu_file(root / "imu.csv");
  std::string line;
  std::getline(imu_file, line);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> fields; std::istringstream header(line); std::string field;
  while (std::getline(header, field, ',')) fields.push_back(field);
  const auto column = [&](const char *name) -> size_t { return std::find(fields.begin(), fields.end(), name) - fields.begin(); };
  const size_t stamp_column = column("stamp_ns"), available_column = column("available_ns");
  const std::array<size_t, 3> acceleration_columns{column("ax"), column("ay"), column("az")};
  const std::array<size_t, 3> angular_columns{column("gx"), column("gy"), column("gz")};
  std::vector<fast_livo2::ImuSample> imu;
  while (std::getline(imu_file, line)) {
    std::istringstream row(line); std::vector<double> values;
    while (std::getline(row, field, ',')) values.push_back(std::stod(field));
    fast_livo2::ImuSample sample;
    sample.stamp_ns = values.at(stamp_column);
    for (int i = 0; i < 3; ++i) {
      sample.linear_acceleration[i] = static_cast<float>(values.at(acceleration_columns[i]) / 9.80665);
      sample.angular_velocity[i] = static_cast<float>(values.at(angular_columns[i]));
    }
    if (sample.stamp_ns >= 4000000000LL)
      events.push_back({static_cast<uint64_t>(values.at(available_column)), events.size(), imu.size(), 'i'});
    imu.push_back(sample);
  }
  std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
    return std::tie(a.time, a.order) < std::tie(b.time, b.order);
  });
  fast_livo2::Mapper mapper(options.core);
  fast_livo2::LivoxScan scan;
  std::vector<float> xyz;
  std::vector<uint32_t> offsets;
  std::vector<uint8_t> lines;
  size_t frames = 0;
  for (const auto &event : events) {
    if (event.kind == 'i') mapper.pushImu(imu[event.index]);
    else if (event.kind == 'c') {
      const auto &record = images[event.index];
      cv::Mat rgb(record.height, record.width, CV_8UC3), bgr;
      if (record.count != rgb.total() * rgb.elemSize()) throw std::runtime_error("Invalid camera record");
      image_file.seekg(record.offset);
      image_file.read(reinterpret_cast<char *>(rgb.data), record.count);
      if (!image_file) throw std::runtime_error("Truncated camera record");
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      mapper.pushImage(record.stamp, std::move(bgr));
    } else {
      const auto &record = lidar[event.index];
      xyz.resize(record.count * 3); offsets.resize(record.count); lines.resize(record.count); scan.points.resize(record.count);
      cloud_file.seekg(record.offset);
      cloud_file.read(reinterpret_cast<char *>(xyz.data()), xyz.size() * sizeof(float));
      cloud_file.read(reinterpret_cast<char *>(offsets.data()), offsets.size() * sizeof(uint32_t));
      cloud_file.read(reinterpret_cast<char *>(lines.data()), lines.size());
      if (!cloud_file) throw std::runtime_error("Truncated LiDAR record");
      for (size_t i = 0; i < scan.points.size(); ++i)
        scan.points[i] = {offsets[i], xyz[3*i], xyz[3*i+1], xyz[3*i+2], 100, 0, lines[i]};
      mapper.pushLivox(record.stamp, scan);
    }
    for (;;) {
      const auto frame = mapper.process();
      if (!frame.consumed()) break;
      if (frame.status != fast_livo2::FrameStatus::Mapped) continue;
      const auto &pose = frame.odometry;
      if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite()) return 1;
      trajectory << fast_livo2::seconds(pose.stamp_ns) << ',' << pose.position.x() << ',' << pose.position.y()
                 << ',' << pose.position.z() << ',' << pose.orientation.x() << ',' << pose.orientation.y()
                 << ',' << pose.orientation.z() << ',' << pose.orientation.w() << '\n';
      twist << fast_livo2::seconds(pose.stamp_ns) << ',' << pose.linear_velocity_body.x()
            << ',' << pose.linear_velocity_body.y() << ',' << pose.linear_velocity_body.z()
            << ',' << pose.angular_velocity_body.x() << ',' << pose.angular_velocity_body.y()
            << ',' << pose.angular_velocity_body.z() << '\n';
      ++frames;
    }
  }
  mapper.finish();
  std::ofstream(output / "summary.json") << "{\"paired_frames\":" << frames
      << ",\"qualified\":" << (frames > .9 * lidar.size() ? "true" : "false") << ",\"clean_shutdown\":true}\n";
  std::cout << "frames=" << frames << '\n';
  return frames ? 0 : 1;
}
