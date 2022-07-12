/**
 * @file Entry point for the node that reads camera data.
 */

#include <libcamera/pixel_format.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdint>
#include <map>
#include <string>

#include "core/libcamera_encoder.hpp"
#include "core/stream_info.hpp"
#include "camera_messenger.hpp"

using image_transport::ImageTransport;

namespace {

// Maps LibCamera pixel formats to ROS pixel formats.
const std::map<libcamera::PixelFormat, std::string> kPixelFormatToEncoding = {
    {libcamera::formats::YUV422, sensor_msgs::image_encodings::YUV422}};

bool PublishEncoded(ImageTransport *transport, const StreamInfo &stream_info,
                    void *buffer, size_t buffer_size, int64_t timestamp_us,
                    uint32_t flags) {
  // Create the message for this image.
  sensor_msgs::Image message;

  message.header.stamp.sec = timestamp_us / 1000000;
  message.header.stamp.nsec = (timestamp_us % 1000000) * 1000;

  message.height = stream_info.height;
  message.width = stream_info.width;
  message.step = stream_info.width * stream_info.stride;
  message.is_bigendian = false;

  // Translate from LibCamera format specifiers to ROS.
  const auto encoding = kPixelFormatToEncoding.find(stream_info.pixel_format);
  ROS_FATAL_STREAM_COND(encoding == kPixelFormatToEncoding.end(),
                        "Got pixel format " << stream_info.pixel_format
                                            << ", which is not supported.");
  message.encoding = encoding->second;

  // I don't get why people still use void pointers in the Year of Our Lord
  // 2022...
  const uint8_t *byte_buffer = static_cast<uint8_t *>(buffer);
  // Copy raw image data.
  message.data.assign(byte_buffer, byte_buffer + buffer_size);
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  ROS_INFO_STREAM("Starting camera node...");
}
