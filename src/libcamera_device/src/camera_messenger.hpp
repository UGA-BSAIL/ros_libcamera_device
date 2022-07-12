#ifndef ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
#define ROS_LIBCAMERA_CAMERA_MESSENGER_HPP

#include <memory>

#include <image_transport/image_transport.h>

#include "core/libcamera_encoder.hpp"

/**
 * @class Translates camera data coming from a `LibcameraApp` to messages
 * that can be published using `ImageTransport`.
 */
class CameraMessenger {
 public:
  /**
   * @param camera_app The camera app to read data from.
   */
  explicit CameraMessenger(std::unique_ptr<LibcameraEncoder>&& camera_app);

  /**
   * @brief Gets the next message from the camera. Will block until a message
   *    is ready.
   * @param message[out] Will be filled in with the message data.
   */
  void GetNextMessage(image_transport::ImageTransport* message);
};

#endif  // ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
