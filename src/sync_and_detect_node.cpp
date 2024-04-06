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

#include <apriltag_detector/apriltag_detector.hpp>
#include <flex_sync/approximate_sync.hpp>
#include <flex_sync/exact_sync.hpp>
#include <flex_sync/live_sync.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tagslam/logging.hpp>

using Image = sensor_msgs::msg::Image;
using Odometry = nav_msgs::msg::Odometry;

namespace flex_sync
{
template <typename SyncT>
class Subscriber<SyncT, Image>
{
public:
  using TConstSharedPtr = typename Image::ConstSharedPtr;

  Subscriber(
    const std::string & topic, rclcpp::Node * node, const rclcpp::QoS & qos,
    const std::shared_ptr<SyncT> & sync)
  : topic_(topic), sync_(sync)
  {
    const std::string param_name = topic + ".image_transport";
    sub_ = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
        node, topic,
        std::bind(
          &Subscriber<SyncT, Image>::callback, this, std::placeholders::_1),
        node->get_parameter_or<std::string>(param_name, "raw"),
        qos.get_rmw_qos_profile()));
  }
  void callback(const TConstSharedPtr & msg) { sync_->process(topic_, msg); }

private:
  std::string topic_;
  std::shared_ptr<SyncT> sync_;
  std::shared_ptr<image_transport::Subscriber> sub_;
};
}  // namespace flex_sync

namespace tagslam
{

class SyncAndDetect : public rclcpp::Node
{
public:
  explicit SyncAndDetect(const rclcpp::NodeOptions & opt)
  : Node("sync_and_detect", opt)
  {
    subscribe();
  }
  void callback2(
    const std::vector<Image::ConstSharedPtr> & imgs,
    const std::vector<Odometry::ConstSharedPtr> & odoms)
  {
    (void)imgs;
    (void)odoms;
    std::cout << "got callback!" << std::endl;
  }

  void callback1(const std::vector<Image::ConstSharedPtr> & imgs)
  {
    std::vector<ApriltagArray> tagMsgs(imgs.size());
    for (size_t i = 0; i < imgs.size(); i++) {
      detector_[i]->detect(imgs[i].get(), tagMsgs[i]->get());
    }
  }

private:
  using ImageOnlyExactSync = flex_sync::LiveSync<flex_sync::ExactSync<Image>>;
  using ImageAndOdomExactSync =
    flex_sync::LiveSync<flex_sync::ExactSync<Image, Odometry>>;
  using ImageOnlyApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image>>;
  using ImageAndOdomApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image, Odometry>>;

  void subscribe()
  {
    std::vector<std::vector<std::string>> topics_1 = {
      {"/cam_sync/cam0/image_raw", "/cam_sync/cam1/image_raw"}};
    for (const auto & topic : topics_1[0]) {
      declare_parameter<std::string>(topic + ".image_transport", "ffmpeg");
    }
    auto cb1 =
      std::bind(&SyncAndDetect::callback1, this, std::placeholders::_1);
    image_only_approx_sync_ =
      std::make_shared<ImageOnlyApproxSync>(this, topics_1, cb1, 10);
    pluginlib::ClassLoader<ApriltagDetector> detector_;
  }
  std::shared_ptr<ImageOnlyExactSync> image_only_exact_sync_;
  std::shared_ptr<ImageAndOdomExactSync> image_and_odom_exact_sync_;
  std::shared_ptr<ImageOnlyApproxSync> image_only_approx_sync_;
  std::shared_ptr<ImageAndOdomApproxSync> image_and_odom_approx_sync_;
};
}  // namespace tagslam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tagslam::SyncAndDetect>(rclcpp::NodeOptions());
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
