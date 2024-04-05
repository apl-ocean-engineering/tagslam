// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_transport/player.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/tagslam.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "tagslam_from_bag -b input_bag -o output_dir -c config_dir"
            << std::endl;
}

using Image = sensor_msgs::msg::Image;
using Path = std::filesystem::path;
using Parameter = rclcpp::Parameter;

namespace tagslam
{

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("tagslam_from_bag"));
}

template <typename T>
static typename T::UniquePtr deserialize(
  const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
  typename T::UniquePtr m(new T());
  rclcpp::Serialization<T> serialization;
  serialization.deserialize_message(&serializedMsg, m.get());
  return (m);
}

class ImageRepublisher : public rclcpp::Node
{
public:
  ImageRepublisher(
    const std::vector<std::string> & topics, const rclcpp::NodeOptions & opt)
  : Node("image_pub", opt)
  {
    for (const auto & topic : topics) {
      pub_.push_back(create_publisher<Image>(topic, 1));
      topic_map_.insert({topic, pub_.back()});
    }
  }
  void publish(const std::string & topic, Image::UniquePtr m)
  {
    auto it = topic_map_.find(topic);
    if (it == topic_map_.end()) {
      LOG_ERROR("bad topic: " << topic);
      throw(std::runtime_error("bad topic"));
    }
    it->second->publish(std::move(m));
  }

private:
  std::vector<rclcpp::Publisher<Image>::SharedPtr> pub_;
  std::unordered_map<std::string, rclcpp::Publisher<Image>::SharedPtr>
    topic_map_;
};

void tagslam_from_bag(
  const std::string & bag, const std::string & out_dir,
  const std::string & cdir)
{
  (void)out_dir;
  const std::vector<std::string> topics = {
    "cam_sync/cam1/image_raw", "cam_sync/cam1/image_raw"};
  rosbag2_storage::StorageOptions storage;
  storage.uri = bag;
  rosbag2_transport::PlayOptions play_options;
  // play_options.topics_to_filter = topics;  no filtering by default
  auto player = std::make_shared<rosbag2_transport::Player>(
    storage, play_options, "tagslam_rosbag_player", rclcpp::NodeOptions());
  rclcpp::NodeOptions opt;
  opt.use_intra_process_comms(true);
  Path cd(cdir);
  opt.parameter_overrides(
    {Parameter("tagslam_config", (cd / Path("tagslam.yaml")).string()),
     Parameter("cameras", (cd / Path("cameras.yaml")).string()),
     Parameter("camera_poses", (cd / Path("camera_poses.yaml")).string())});
  auto tagslam = std::make_shared<TagSLAM>(opt);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(player);
  executor.add_node(tagslam);
  for (int i = 0; i < 1000 && rclcpp::ok(); i++) {
    LOG_INFO("spinning");
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace tagslam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int opt;
  std::string inFile;
  std::string outDir;
  std::string configDir;
  while ((opt = getopt(argc, argv, "b:o:c:h")) != -1) {
    switch (opt) {
      case 'b':
        inFile = optarg;
        break;
      case 'o':
        outDir = optarg;
        break;
      case 'c':
        configDir = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;

      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outDir.empty() || configDir.empty()) {
    std::cout << "missing input/output/config name!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  tagslam::tagslam_from_bag(inFile, outDir, configDir);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for running tagslam: "
            << total_duration.count() * 1e-6 << std::endl;
  return (0);
}
