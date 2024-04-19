// -*-c++-*---------------------------------------------------------------------------------------
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

#include <rosbag2_transport/recorder.hpp>
#include <tagslam/enhanced_player.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/tagslam.hpp>

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("tagslam_from_bag"));
}

static void printTopics(
  const std::string & label, const std::vector<std::string> & topics)
{
  for (const auto & t : topics) {
    LOG_INFO(label << ": " << t);
  }
}
static std::vector<std::string> merge(
  const std::vector<std::string> & v1, const std::vector<std::string> & v2)
{
  std::vector<std::string> all = v1;
  all.insert(all.end(), v2.begin(), v2.end());
  return (all);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  // node_options.automatically_declare_parameters_from_overrides(true);

  auto tagslam_node = std::make_shared<tagslam::TagSLAM>(node_options);
  exec.add_node(tagslam_node);

  const auto in_topics =
    merge(tagslam_node->getTagTopics(), tagslam_node->getOdomTopics());
  printTopics("taglslam input topics", in_topics);

  const auto recorded_topics = tagslam_node->getPublishedTopics();
  printTopics("recorded topic", recorded_topics);

  const std::string in_uri =
    tagslam_node->declare_parameter<std::string>("in_bag", "");
  if (in_uri.empty()) {
    LOG_ERROR("must provide valid in_bag parameter!");
    return (-1);
  }
  LOG_INFO("using input bag: " << in_uri);
  rclcpp::NodeOptions player_options;
  using Parameter = rclcpp::Parameter;
  player_options.parameter_overrides(
    {Parameter("storage.uri", in_uri), Parameter("play.topics", in_topics),
     Parameter("play.clock_publish_on_topic_publish", true),
     Parameter("play.start_paused", true), Parameter("play.rate", 1000.0),
     Parameter("play.disable_keyboard_controls", true)});
  auto player_node =
    std::make_shared<tagslam::EnhancedPlayer>("rosbag_player", player_options);
  player_node->get_logger().set_level(rclcpp::Logger::Level::Warn);
  exec.add_node(player_node);

  if (!player_node->hasAllTopics(in_topics)) {
    LOG_ERROR("topics not in bag, tagslam will hang!");
  }

  const std::string out_uri =
    tagslam_node->declare_parameter<std::string>("out_bag", "");

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node;
  if (!out_uri.empty()) {
    LOG_INFO("writing detected tags to bag: " << out_uri);
    rclcpp::NodeOptions recorder_options;
    recorder_options.parameter_overrides(
      {Parameter("storage.uri", out_uri),
       Parameter("record.disable_keyboard_controls", true),
       Parameter("record.topics", recorded_topics)});

    recorder_node = std::make_shared<rosbag2_transport::Recorder>(
      "rosbag_recorder", recorder_options);
    exec.add_node(recorder_node);
  } else {
    LOG_INFO("no out_bag parameter set, publishing as messages!");
  }

  while (player_node->play_next() && rclcpp::ok()) {
    exec.spin_some();
  }
  tagslam_node->finalize();

  rclcpp::shutdown();
  return 0;
}
