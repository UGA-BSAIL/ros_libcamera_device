/**
 * @file Entry point for the node that reads camera data.
 */

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <libcamera_device/LibcameraDeviceConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cstdint>
#include <functional>
#include <memory>

#include "camera_messenger.hpp"
#include "core/libcamera_encoder.hpp"
#include "core/video_options.hpp"

using dynamic_reconfigure::Server;
using image_transport::ImageTransport;
using image_transport::Publisher;
using libcamera_device::CameraMessenger;
using libcamera_device::LibcameraDeviceConfig;
using sensor_msgs::Image;

namespace {

// Unfortunately, Boost has its own placeholders that conflict with the std
// ones, so we have to rename them.
const auto kStd1 = std::placeholders::_1;
const auto kStd2 = std::placeholders::_2;

/**
 * Publishes a camera message. This is meant to be used as a callback.
 * @param publisher Will be used for publishing images.
 * @param image The image to publish.
 */
void PublishEncoded(Publisher *publisher, const Image &image) {
  publisher->publish(image);
}

/**
 * @brief Translates the configuration struct used by `dynamic_reconfigure` to
 *  the one used by `libcamera`.
 * @param in_config The parameter configuration.
 * @param out_config [out] The camera configuration.
 */
void ParamToVideoConfig(const LibcameraDeviceConfig &in_config,
                        VideoOptions *out_config) {
  out_config->framerate = in_config.fps;
  out_config->mode = Mode(in_config.width, in_config.height, 24, false);

  // Set options that are necessary, but that we don't support configuring
  // (yet). We don't have any use for preview mode.
  out_config->nopreview = true;
  // Use automatic denoising.
  out_config->denoise = "off";
  // This just outputs the raw image data with no encoding.
  out_config->codec = "yuv420";

  out_config->brightness = 0.0;
  out_config->contrast = 1.0;
  out_config->saturation = 1.0;
  out_config->sharpness = 1.0;
}

/**
 * @brief Reconfigures the camera parameters. Meant to be used as a callback for
 *   dynamic reconfiguration.
 * @param messenger The `CameraMessenger` to reconfigure.
 * @param config The configuration that was changed.
 * @param level The configuration level bitmask.
 */
void ReconfigureParams(CameraMessenger *messenger,
                       LibcameraDeviceConfig &config, uint32_t level) {
  ROS_INFO_STREAM("Reconfigure request: " << config.width << "x"
                                          << config.height << ", " << config.fps
                                          << " FPS.");

  // Set the new parameters.
  VideoOptions camera_options;
  ParamToVideoConfig(config, &camera_options);

  // Camera needs to be restarted for these to take effect.
  messenger->Stop();
  messenger->ConfigureOptions(camera_options);
  messenger->Start();
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  ROS_INFO_STREAM("Starting camera node...");

  ImageTransport image_transport(node);
  auto image_publisher = image_transport.advertise("camera", 1);

  Server<LibcameraDeviceConfig> param_server;

  // Set the default configuration initially.
  LibcameraDeviceConfig default_param_config;
  VideoOptions default_video_options;
  param_server.getConfigDefault(default_param_config);
  ParamToVideoConfig(default_param_config, &default_video_options);

  // Set up the camera.
  CameraMessenger camera(std::make_unique<LibcameraEncoder>(), "camera_frame",
                         default_video_options);
  camera.SetMessageReadyCallback(
      std::bind(PublishEncoded, &image_publisher, kStd1));
  camera.Start();

  // Configure the dynamic reconfiguration reconfigure_callback.
  Server<LibcameraDeviceConfig>::CallbackType reconfigure_callback =
      std::bind(ReconfigureParams, &camera, kStd1, kStd2);
  param_server.setCallback(reconfigure_callback);

  // Process all camera messages.
  while (camera.WaitForFrame()) {
    ros::spinOnce();
  }

  return 0;
}
