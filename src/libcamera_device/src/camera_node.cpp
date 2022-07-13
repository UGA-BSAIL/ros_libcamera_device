/**
 * @file Entry point for the node that reads camera data.
 */

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <functional>
#include <memory>

#include "camera_messenger.hpp"
#include "core/libcamera_encoder.hpp"

using image_transport::ImageTransport;
using image_transport::Publisher;
using libcamera_device::CameraMessenger;
using sensor_msgs::Image;

namespace {

// Unfortunately, Boost has its own placeholders that conflict with the std
// ones, so we have to rename them.
const auto kStd1 = std::placeholders::_1;

/**
 * Publishes a camera message. This is meant to be used as a callback.
 * @param publisher Will be used for publishing images.
 * @param image The image to publish.
 */
void PublishEncoded(Publisher *publisher, const Image &image) {
  publisher->publish(image);
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  ROS_INFO_STREAM("Starting camera node...");

  ImageTransport image_transport(node);
  auto image_publisher = image_transport.advertise("camera", 1);

  // Set up the camera.
  CameraMessenger camera(std::make_unique<LibcameraEncoder>(), "camera_frame");
  camera.SetMessageReadyCallback(
      std::bind(PublishEncoded, &image_publisher, kStd1));

  // Process all camera messages.
  camera.Run();

  return 0;
}
