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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <apriltag_detector/detector.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <flex_sync/approximate_sync.hpp>
#include <flex_sync/exact_sync.hpp>
#include <flex_sync/live_sync.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tagslam/logging.hpp>

using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
using Image = sensor_msgs::msg::Image;
using Odometry = nav_msgs::msg::Odometry;
using Parameter = rclcpp::Parameter;

using Path = std::filesystem::path;
using std::placeholders::_1;
using std::placeholders::_2;

template <class T>
using vector = std::vector<T>;
using std::string;
using svec = vector<string>;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout
    << "sync_and_detect_from_bag -b input_bag -o output_bag -c config_dir"
    << std::endl;
}

namespace flex_sync
{
/*
* Specialize the Subscriber template in the flex_sync namespace to use
* the image_transport for subscribing instead of the regular transport.
*/
template <typename SyncT>
class Subscriber<SyncT, Image>
{
public:
  using TConstSharedPtr = typename Image::ConstSharedPtr;

  Subscriber(
    const string & topic, rclcpp::Node * node, const rclcpp::QoS & qos,
    const std::shared_ptr<SyncT> & sync)
  : topic_(topic), sync_(sync)
  {
    const string param_name = topic + ".image_transport";
    sub_ = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
        node, topic, std::bind(&Subscriber<SyncT, Image>::callback, this, _1),
        node->get_parameter_or<string>(param_name, "raw"),
        qos.get_rmw_qos_profile()));
  }
  ~Subscriber()
  {
    sub_->shutdown();
    std::cout << "image subscriber shut down" << std::endl;
  }
  void callback(const TConstSharedPtr & msg) { sync_->process(topic_, msg); }

private:
  string topic_;
  std::shared_ptr<SyncT> sync_;
  std::shared_ptr<image_transport::Subscriber> sub_;
};
}  // namespace flex_sync

namespace tagslam
{

class SyncAndDetectListener
{
public:
  virtual ~SyncAndDetectListener()
  {
    std::cout << "destroyed base class s&dl" << std::endl;
  }
  virtual void callbackTags(
    const vector<ApriltagArray::ConstSharedPtr> & tagMsgs) = 0;
  virtual void callbackTagsAndOdom(
    const vector<ApriltagArray::ConstSharedPtr> & tagMsgs,
    const vector<Odometry::ConstSharedPtr> & odoms) = 0;
};

class Publisher : public SyncAndDetectListener
{
public:
  Publisher(
    rclcpp::Node * node, const svec & tag_topics, const svec & odom_topics)
  {
    for (const auto & tt : tag_topics) {
      tag_pub_.push_back(node->create_publisher<ApriltagArray>(tt, 10));
    }
    for (const auto & ot : odom_topics) {
      odom_pub_.push_back(node->create_publisher<Odometry>(ot, 10));
    }
  }
  ~Publisher()
  {
    std::cout << "destroying publisher" << std::endl;
    tag_pub_.clear();
    odom_pub_.clear();
    std::cout << "destroyed publisher" << std::endl;
  }

  void callbackTags(const vector<ApriltagArray::ConstSharedPtr> & tagMsgs) final
  {
    publishTags(tagMsgs);
  }

  void callbackTagsAndOdom(
    const vector<ApriltagArray::ConstSharedPtr> & tagMsgs,
    const vector<Odometry::ConstSharedPtr> & odoms) final
  {
    publishTags(tagMsgs);
    for (size_t i = 0; i < odoms.size(); i++) {
      odom_pub_[i]->publish(*odoms[i]);
    }
  }

private:
  void publishTags(const vector<ApriltagArray::ConstSharedPtr> & tagMsgs)
  {
    for (size_t i = 0; i < tagMsgs.size(); i++) {
      tag_pub_[i]->publish(*tagMsgs[i]);
    }
  }
  // ---------------- variables -------------------
  vector<rclcpp::Publisher<ApriltagArray>::SharedPtr> tag_pub_;
  vector<rclcpp::Publisher<Odometry>::SharedPtr> odom_pub_;
};

class SyncAndDetect : public rclcpp::Node
{
public:
  explicit SyncAndDetect(const rclcpp::NodeOptions & opt)
  : Node("sync_and_detect", opt),
    detector_loader_("apriltag_detector", "apriltag_detector::Detector")
  {
#if 0    
    auto
      [img_topics, tag_pub_topics, transports, detectors, odom_topics,
       odom_pub_topics] = getTopics();
#else
    const svec img_topics = {
      "/cam_sync/cam_0/image_raw", "/cam_sync/cam_1/image_raw"};
    const svec odom_topics;
    const svec transports = {"raw", "raw"};
    const svec detectors = {"umich", "umich"};
    const svec odom_pub_topics;
#endif
    if (get_parameter_or<bool>("publish", true)) {
      // pub_ = std::make_shared<Publisher>(this, tag_pub_topics, odom_pub_topics);
      // listener_ = pub_; // XXX
    }

    subscribe({img_topics, odom_topics}, transports, detectors);
  }

  ~SyncAndDetect()
  {
    image_exact_sync_.reset();
    image_approx_sync_.reset();
    image_odom_exact_sync_.reset();
    image_odom_approx_sync_.reset();
    detectors_.clear();
    pub_.reset();
    detector_loader_.unloadLibraryForClass("apriltag_detector_umich::Detector");
    std::cout << "destroyed sync-and-detect" << std::endl;
  }

  void setListener(const std::shared_ptr<SyncAndDetectListener> & m)
  {
    listener_ = m;
  }

  std::tuple<svec, svec, svec, svec, svec, svec> getTopics()
  {
    svec odom, odom_pub;
    auto [img, tag, transport, detector] =
      parseCameras(readConfig(getParameter("cameras")));
    return {img, tag, transport, detector, odom, odom_pub};
  }

  std::tuple<svec, svec, svec, svec> parseCameras(const YAML::Node & conf)
  {
    svec img, tag, transport, detector;
    svec cam_names;
    for (const auto & c : conf) {
      const auto name = c.first.as<std::string>();
      if (name.find("cam") != string::npos) {
        cam_names.push_back(name);
      }
    }
    std::sort(cam_names.begin(), cam_names.end(), [](string a, string b) {
      return a < b;
    });
    for (const auto & cam : cam_names) {
      const auto & c = conf[cam];
      if (!c["image_topic"] || !c["tag_topic"]) {
        BOMB_OUT("must specify image_topic and tag_topic for camera " << cam);
      }
      img.push_back(c["image_topic"].as<string>());
      tag.push_back(c["tag_topic"].as<string>());
      transport.push_back(
        c["image_transport"] ? c["image_transport"].as<string>() : "raw");
      detector.push_back(
        c["tag_detector"] ? c["tag_detector"].as<string>() : "umich");
    }
    return {img, tag, transport, detector};
  }

  string getParameter(const string & name)
  {
    const string p = get_parameter_or<string>(name, "");
    if (p.empty()) {
      BOMB_OUT("parameter " << name << " must be specified and non-empty!");
    }
    return (p);
  }

  YAML::Node readConfig(const std::string & conf_file)
  {
    YAML::Node config = YAML::LoadFile(conf_file);
    if (config.IsNull()) {
      BOMB_OUT("cannot open config file: " << conf_file);
    }
    return (config);
  }

  void callbackImageAndOdom(
    const vector<Image::ConstSharedPtr> & imgs,
    const vector<Odometry::ConstSharedPtr> & odoms)
  {
    vector<ApriltagArray::ConstSharedPtr> tagMsgs;
    num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
    if (listener_) {
      listener_->callbackTagsAndOdom(tagMsgs, odoms);
    }
  }

  void callbackImage(const vector<Image::ConstSharedPtr> & imgs)
  {
    std::cout << "callback image: " << imgs.size() << std::endl;
    vector<ApriltagArray::ConstSharedPtr> tagMsgs;
    num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
    std::cout << "num tags detected: " << num_tags_detected_ << std::endl;
    if (listener_) {
      listener_->callbackTags(tagMsgs);
    }
  }

  size_t getNumberOfTagsDetected() const { return (num_tags_detected_); }

private:
  using ImageExactSync = flex_sync::LiveSync<flex_sync::ExactSync<Image>>;
  using ImageAndOdomExactSync =
    flex_sync::LiveSync<flex_sync::ExactSync<Image, Odometry>>;
  using ImageApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image>>;
  using ImageAndOdomApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image, Odometry>>;

  void subscribe(
    const vector<svec> & topics, const svec & transports,
    const svec & detectors)
  {
    assert(topics.size() == 1 || topics.size() == 2);
    const auto & imgs = topics[0];
    assert(imgs.size() == transports.size());
    assert(imgs.size() == detectors.size());

    for (size_t i = 0; i < imgs.size(); i++) {
      const auto & topic = imgs[i];
      declare_parameter<string>(topic + ".image_transport", transports[i]);
      std::cout << "making detector of type: " << detectors[i] << std::endl;
      detectors_.push_back(detector_loader_.createSharedInstance(
        "apriltag_detector_" + detectors[i] + "::Detector"));
    }
    const bool use_approx_sync =
      get_parameter_or<bool>("use_approximate_sync", true);
    LOG_INFO("using approximate sync: " << (use_approx_sync ? "YES" : "NO"));
    const int qs = 10;  // queue size
    if (topics.size() == 1) {
      if (use_approx_sync) {
        image_approx_sync_ = std::make_shared<ImageApproxSync>(
          this, topics, std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
      } else {
        image_exact_sync_ = std::make_shared<ImageExactSync>(
          this, topics, std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
      }
    } else {
      if (use_approx_sync) {
        image_odom_approx_sync_ = std::make_shared<ImageAndOdomApproxSync>(
          this, topics, std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
      } else {
        image_odom_exact_sync_ = std::make_shared<ImageAndOdomExactSync>(
          this, topics, std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
      }
    }
  }

  size_t tagsFromImages(
    const vector<Image::ConstSharedPtr> & imgs,
    vector<ApriltagArray::ConstSharedPtr> * tagMsgs)
  {
    assert(detectors_.size() == imgs.size());
    size_t num_tags{0};
    for (size_t i = 0; i < imgs.size(); i++) {
      const auto & img = imgs[i];
      auto tags = std::make_shared<ApriltagArray>();
      tagMsgs->push_back(tags);
      tags->header = img->header;
#if 0
      cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "mono8");
      if (!cvImg) {
        BOMB_OUT("cannot convert image to mono!");
      }
#endif
      detectors_[i]->detect(img, tags.get());
      num_tags += tags->detections.size();
    }
    return (num_tags);
  }

  // ----------------- variables ----------------------------------
  pluginlib::ClassLoader<apriltag_detector::Detector> detector_loader_;
  vector<std::shared_ptr<apriltag_detector::Detector>> detectors_;
  size_t num_tags_detected_{0};
  std::shared_ptr<SyncAndDetectListener> listener_;
  std::shared_ptr<Publisher> pub_;
  std::shared_ptr<ImageExactSync> image_exact_sync_;
  std::shared_ptr<ImageAndOdomExactSync> image_odom_exact_sync_;
  std::shared_ptr<ImageApproxSync> image_approx_sync_;
  std::shared_ptr<ImageAndOdomApproxSync> image_odom_approx_sync_;
};
}  // namespace tagslam

int main(int argc, char ** argv)
{
  int opt;
  std::string inBag;
  std::string configDir;
  std::string outBag;
  while ((opt = getopt(argc, argv, "b:c:o:h")) != -1) {
    switch (opt) {
      case 'c':
        configDir = optarg;
        break;
      case 'b':
        inBag = optarg;
        break;
      case 'o':
        outBag = optarg;
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
  if (inBag.empty() || outBag.empty() || configDir.empty()) {
    std::cout << "missing inbag/outbag/config_dir" << std::endl;
    usage();
    return (-1);
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_opt;
  Path cd(configDir);
  node_opt.parameter_overrides(
    {Parameter("tagslam_config", (cd / Path("tagslam.yaml")).string()),
     Parameter("cameras", (cd / Path("cameras.yaml")).string()),
     Parameter("use_approximate_sync", true)});

  auto node = std::make_shared<tagslam::SyncAndDetect>(
    node_opt.automatically_declare_parameters_from_overrides(true));
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
