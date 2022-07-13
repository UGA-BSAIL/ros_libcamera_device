#include "camera_messenger.hpp"

#include <libcamera/pixel_format.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <map>
#include <utility>

namespace libcamera_device {
namespace {

// Unfortunately, Boost has its own placeholders that conflict with the std
// ones, so we have to rename them.
const auto kStd1 = std::placeholders::_1;
const auto kStd2 = std::placeholders::_2;
const auto kStd3 = std::placeholders::_3;
const auto kStd4 = std::placeholders::_4;

// Maps LibCamera pixel formats to ROS pixel formats.
const std::map<libcamera::PixelFormat, std::string> kPixelFormatToEncoding = {
    {libcamera::formats::YUV422, sensor_msgs::image_encodings::YUV422},
    {libcamera::formats::RGB888, sensor_msgs::image_encodings::RGB8},
};

}  // namespace

CameraMessenger::CameraMessenger(std::unique_ptr<LibcameraEncoder>&& camera_app,
                                 std::string frame_id)
    : camera_app_(std::move(camera_app)),
      frame_id_(std::move(frame_id)),
      on_message_ready_([](const sensor_msgs::Image&) {
        // Default callback does nothing, but logs a warning.
        ROS_WARN_STREAM("Got a camera message, but no callback is registered.");
      }) {
  // Set sane default options.
  ConfigureOptions();
}

void CameraMessenger::TranslateEncoded(void* buffer, size_t buffer_size,
                                       int64_t timestamp_us, uint32_t) {
  // Create the message for this image.
  sensor_msgs::Image message;

  message.header.stamp.sec = timestamp_us / 1000000;
  message.header.stamp.nsec = (timestamp_us % 1000000) * 1000;
  message.header.seq = message_sequence_++;
  message.header.frame_id = frame_id_;

  message.height = stream_info_.height;
  message.width = stream_info_.width;
  message.step = stream_info_.stride;
  message.is_bigendian = false;
  message.encoding = ros_pixel_format_;

  // I don't get why people still use void pointers in the Year of Our Lord
  // 2022...
  const uint8_t* byte_buffer = static_cast<uint8_t*>(buffer);
  // Copy raw image data.
  message.data.assign(byte_buffer, byte_buffer + buffer_size);

  // Run the callback with the new message.
  on_message_ready_(message);
}

void CameraMessenger::SetMessageReadyCallback(
    const CameraMessenger::MessageReadyCallback& callback) {
  ROS_DEBUG_STREAM("Setting new callback for camera messages.");
  on_message_ready_ = callback;
}

void CameraMessenger::Run() {
  ROS_INFO_STREAM("Starting camera.");

  // Set up the callback.
  camera_app_->SetEncodeOutputReadyCallback(std::bind(
      &CameraMessenger::TranslateEncoded, this, kStd1, kStd2, kStd3, kStd4));

  camera_app_->OpenCamera();
  camera_app_->ConfigureVideo(LibcameraEncoder::FLAG_VIDEO_NONE);

  // Stream info isn't available until after the camera is configured.
  UpdateStreamInfo();

  camera_app_->StartEncoder();
  camera_app_->StartCamera();

  // Main event loop.
  while (true) {
    LibcameraEncoder::Msg message = camera_app_->Wait();
    if (message.type == LibcameraEncoder::MsgType::Quit) {
      ROS_INFO_STREAM("Stopping camera due to LibCamera request.");
      break;
    }
    ROS_FATAL_STREAM_COND(
        message.type != LibcameraEncoder::MsgType::RequestComplete,
        "Got unrecognized message type " << static_cast<uint32_t>(message.type)
                                         << " from LibCamera!");

    auto& completed_request = std::get<CompletedRequestPtr>(message.payload);
    camera_app_->EncodeBuffer(completed_request, camera_app_->VideoStream());
  }

  ROS_INFO_STREAM("Stopping camera.");
  camera_app_->StopCamera();
  camera_app_->StopEncoder();
}

void CameraMessenger::UpdateStreamInfo() {
  stream_info_ = camera_app_->GetStreamInfo(camera_app_->VideoStream());

  // Set the pixel format correctly.
  const auto encoding = kPixelFormatToEncoding.find(stream_info_.pixel_format);
  ROS_FATAL_STREAM_COND(encoding == kPixelFormatToEncoding.end(),
                        "Got pixel format " << stream_info_.pixel_format
                                            << ", which is not supported.");
  ros_pixel_format_ = encoding->second;
}

void CameraMessenger::ConfigureOptions() {
  auto *options = camera_app_->GetOptions();

  // We don't have any use for preview mode.
  options->nopreview = true;
  // Use automatic denoising.
  options->denoise = "auto";
  // This just outputs the raw image data with no encoding.
  options->codec = "yuv420";
}

}  // namespace libcamera_device
