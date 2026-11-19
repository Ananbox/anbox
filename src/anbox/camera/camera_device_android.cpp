/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted for Anbox by Yuliang Tan 2026.
 */

/*
 * Contains code that is used to capture video frames from a camera device on
 * Android. This code uses the NDK camera2 API (libcamera2ndk) to work with
 * camera devices.
 *
 * Unlike the V4L2 backend, the NDK API is a push model: frames are delivered
 * to an AImageReader callback on a binder thread owned by the camera service.
 * The pull based CameraDevice::readFrame() interface is mapped onto it by
 * keeping a copy of the most recent frame (see "capture buffer management" in
 * the implementation notes for AndroidCameraDevice).
 */

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <camera/NdkCameraError.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCaptureRequest.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraMetadataTags.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include "anbox/do_not_copy_or_move.h"
#include "anbox/logger.h"
#include "anbox/camera/camera_device.h"
#include "anbox/camera/camera_format_converters.h"

#define  E(...)    ERROR(__VA_ARGS__)
#define  W(...)    WARNING(__VA_ARGS__)
#define  D(...)    DEBUG(__VA_ARGS__)

/*
 * Camera identifiers in the NDK API are opaque strings handed out by the
 * camera manager. Only the backend knows about them, the rest of anbox
 * treats them as opaque.
 */
namespace anbox::camera {

namespace {

/* Pixel format reported to the guest.
 *
 * The guest only accepts YV12 (V4L2_PIX_FMT_YVU420), NV12 and NV21 for video
 * and requires RGB32 for preview (see camera_message_processor.cpp). These
 * constants must stay in sync with the V4L2 fourcc values, which is also what
 * the converter table in camera_format_converters.cpp is keyed on.
 *
 * This is a copy of the V4L2 fourcc macro, <linux/videodev2.h> is not
 * available on Android.
 */
#ifndef A_ANBOX_FOURCC
#define A_ANBOX_FOURCC(a, b, c, d) \
  ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#endif

/* Guest facing YV12. */
static const uint32_t kGuestPixelFormat = A_ANBOX_FOURCC('Y', 'V', '1', '2');

/* Frame sizes we fall back to when the device characteristics do not provide
 * a usable list. 640x480 is guaranteed by the camera2 API for every device
 * that can produce YUV output. */
struct Size { int width; int height; };
static const Size kDefaultSizes[] = {
  { 640, 480 },
  { 320, 240 },
};

/* Returns a printable form of a camera_status_t. */
const char* status_to_string(camera_status_t status) {
  switch (status) {
    case ACAMERA_OK: return "ACAMERA_OK";
    case ACAMERA_ERROR_UNKNOWN: return "ACAMERA_ERROR_UNKNOWN";
    case ACAMERA_ERROR_INVALID_PARAMETER: return "ACAMERA_ERROR_INVALID_PARAMETER";
    case ACAMERA_ERROR_CAMERA_DISCONNECTED: return "ACAMERA_ERROR_CAMERA_DISCONNECTED";
    case ACAMERA_ERROR_NOT_ENOUGH_MEMORY: return "ACAMERA_ERROR_NOT_ENOUGH_MEMORY";
    case ACAMERA_ERROR_METADATA_NOT_FOUND: return "ACAMERA_ERROR_METADATA_NOT_FOUND";
    case ACAMERA_ERROR_CAMERA_DEVICE: return "ACAMERA_ERROR_CAMERA_DEVICE";
    case ACAMERA_ERROR_CAMERA_SERVICE: return "ACAMERA_ERROR_CAMERA_SERVICE";
    case ACAMERA_ERROR_SESSION_CLOSED: return "ACAMERA_ERROR_SESSION_CLOSED";
    case ACAMERA_ERROR_INVALID_OPERATION: return "ACAMERA_ERROR_INVALID_OPERATION";
    case ACAMERA_ERROR_STREAM_CONFIGURE_FAIL: return "ACAMERA_ERROR_STREAM_CONFIGURE_FAIL";
    case ACAMERA_ERROR_CAMERA_IN_USE: return "ACAMERA_ERROR_CAMERA_IN_USE";
    case ACAMERA_ERROR_MAX_CAMERA_IN_USE: return "ACAMERA_ERROR_MAX_CAMERA_IN_USE";
    case ACAMERA_ERROR_CAMERA_DISABLED: return "ACAMERA_ERROR_CAMERA_DISABLED";
    case ACAMERA_ERROR_PERMISSION_DENIED: return "ACAMERA_ERROR_PERMISSION_DENIED";
    default: return "unknown camera error";
  }
}

/* Reads an int32 entry, returns false when the tag is missing. */
bool metadata_get_int32(ACameraMetadata* md, uint32_t tag, int32_t* out) {
  ACameraMetadata_const_entry entry;
  if (ACameraMetadata_getConstEntry(md, tag, &entry) != ACAMERA_OK) {
    return false;
  }
  if (entry.type != ACAMERA_TYPE_INT32 || entry.count < 1) {
    return false;
  }
  *out = entry.data.i32[0];
  return true;
}

/* Builds the list of frame sizes for the given YUV format.
 *
 * The camera2 API enumerates every output format/size combination in
 * ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS. Sizes are reported as
 * width, height, format, isInput quads. Only capture (isInput == 0) sizes in
 * one of the guest supported YUV formats are collected, and since the format
 * we finally report is YV12 anyway, all of them are equivalent for the guest.
 */
std::shared_ptr<std::vector<CameraFrameDim>>
collect_frame_sizes(ACameraMetadata* md) {
  auto result = std::make_shared<std::vector<CameraFrameDim>>();

  ACameraMetadata_const_entry entry;
  if (ACameraMetadata_getConstEntry(
          md, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry) != ACAMERA_OK) {
    return result;
  }
  if (entry.type != ACAMERA_TYPE_INT32 || entry.count < 4) {
    return result;
  }

  for (uint32_t i = 0; i + 3 < entry.count; i += 4) {
    const int32_t w = entry.data.i32[i + 0];
    const int32_t h = entry.data.i32[i + 1];
    const int32_t fmt = entry.data.i32[i + 2];
    const int32_t is_input = entry.data.i32[i + 3];

    if (is_input != 0 || w <= 0 || h <= 0) {
      continue;
    }
    /* AIMAGE_FORMAT_YUV_420_888 lives in the HAL pixel format space. We only
     * need to know that *some* YUV output exists, the actual conversion is
     * done on the ImageReader side. */
    if (fmt != AIMAGE_FORMAT_YUV_420_888) {
      continue;
    }

    bool duplicate = false;
    for (const auto& d : *result) {
      if (d.width == w && d.height == h) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }

    CameraFrameDim dim;
    dim.width = w;
    dim.height = h;
    result->push_back(dim);
  }

  return result;
}

}  // namespace

/*******************************************************************************
 *                     AndroidCameraDevice
 ******************************************************************************/

/*
 * Describes a connection to an actual camera device.
 */
class AndroidCameraDevice : public CameraDevice {
 public:
  AndroidCameraDevice(std::string camera_id, int input_channel);
  ~AndroidCameraDevice() override;

  /* Opens the camera. Must be called once before startCapturing().
   * Return:
   *  0 on success, != 0 on failure.
   */
  int openDevice();

  /* Collects information about a camera without keeping it open.
   * Param:
   *  camera_id - ID reported by the camera manager.
   *  ci - Upon success contains information collected from the device.
   * Return:
   *  0 on success, != 0 on failure.
   */
  static int getInfo(const std::string& camera_id, CameraInfo* ci);

  // Inherited interface
  int startCapturing(uint32_t pixel_format,
                     unsigned int frame_width,
                     unsigned int frame_height) override;
  int stopCapturing() override;
  int readFrame(ClientFrameBuffer* framebuffers,
                int fbs_num,
                float r_scale,
                float g_scale,
                float b_scale,
                float exp_comp) override;
  void closeCameraDevice() override;

 private:
  enum class State { kClosed, kOpened };

  /* Releases every NDK resource held by this device. Idempotent. */
  void releaseAll();

  /* Camera id as reported by the NDK camera manager. Owned copy, the list the
   * id came from is invalidated by deleteCameraIdList(). */
  std::string       camera_id_;
  int               input_channel_;

  /*
   * Set after openDevice().
   */

  /* Manager handle. Kept for the lifetime of the device because the camera
   * device and its callbacks are tied to it. */
  ACameraManager*   manager_;
  /* Handle to the opened camera device. */
  ACameraDevice*    device_;
  /* Device state callbacks. Value member: the NDK keeps a pointer to it, so
   * it must outlive ACameraDevice_close(). */
  ACameraDevice_StateCallbacks device_cb_;

  State             state_;

  /*
   * Set after startCapturing().
   *
   * Kept here from the start so the object layout does not change when the
   * capture path is implemented.
   */
  ACameraCaptureSession*          session_;
  ACaptureRequest*                capture_request_;
  ACameraOutputTarget*            output_target_;
  ACaptureSessionOutput*          session_output_;
  ACaptureSessionOutputContainer* output_container_;
  AImageReader*                   image_reader_;
  ANativeWindow*                  image_reader_window_;

  /* Pixel format required by the guest and frame geometry of the reader. */
  uint32_t          pixel_format_;
  int               frame_width_;
  int               frame_height_;
};

AndroidCameraDevice::AndroidCameraDevice(std::string camera_id, int input_channel)
    : camera_id_(std::move(camera_id)),
      input_channel_(input_channel),
      manager_(nullptr),
      device_(nullptr),
      device_cb_{},
      state_(State::kClosed),
      session_(nullptr),
      capture_request_(nullptr),
      output_target_(nullptr),
      session_output_(nullptr),
      output_container_(nullptr),
      image_reader_(nullptr),
      image_reader_window_(nullptr),
      pixel_format_(kGuestPixelFormat),
      frame_width_(0),
      frame_height_(0) {}

AndroidCameraDevice::~AndroidCameraDevice() {
  closeCameraDevice();
}

void AndroidCameraDevice::releaseAll() {
  /* Ownership rules of the NDK:
   *  - ACameraCaptureSession_close() is asynchronous; resources attached to
   *    the session are not valid afterwards.
   *  - ACameraDevice_close() must come after every capture session of that
   *    device has been closed.
   *  - ACameraManager_delete() must come last.
   * Every pointer is cleared so the routine is safe to call twice.
   */
  if (session_ != nullptr) {
    ACameraCaptureSession_close(session_);
    session_ = nullptr;
  }

  if (capture_request_ != nullptr) {
    ACaptureRequest_free(capture_request_);
    capture_request_ = nullptr;
  }

  if (output_target_ != nullptr) {
    ACameraOutputTarget_free(output_target_);
    output_target_ = nullptr;
  }

  if (session_output_ != nullptr) {
    ACaptureSessionOutput_free(session_output_);
    session_output_ = nullptr;
  }

  if (output_container_ != nullptr) {
    ACaptureSessionOutputContainer_free(output_container_);
    output_container_ = nullptr;
  }

  if (image_reader_ != nullptr) {
    AImageReader_delete(image_reader_);
    image_reader_ = nullptr;
  }

  if (image_reader_window_ != nullptr) {
    /* The reader does not own the window it was created from. */
    ANativeWindow_release(image_reader_window_);
    image_reader_window_ = nullptr;
  }

  if (device_ != nullptr) {
    ACameraDevice_close(device_);
    device_ = nullptr;
  }

  if (manager_ != nullptr) {
    ACameraManager_delete(manager_);
    manager_ = nullptr;
  }

  state_ = State::kClosed;
}

int AndroidCameraDevice::openDevice() {
  if (state_ != State::kClosed) {
    E("Camera '%s' is already opened", camera_id_.c_str());
    return -1;
  }

  manager_ = ACameraManager_create();
  if (manager_ == nullptr) {
    E("Unable to create camera manager for camera '%s'", camera_id_.c_str());
    return -1;
  }

  /* The NDK stores the pointer, not a copy of the callbacks. No locks are held
   * here and the callbacks do not touch this object yet, the capture path
   * installs them in startCapturing().
   */
  device_cb_.context = this;
  device_cb_.onDisconnected = nullptr;
  device_cb_.onError = nullptr;

  const camera_status_t status =
      ACameraManager_openCamera(manager_, camera_id_.c_str(), &device_cb_, &device_);
  if (status != ACAMERA_OK || device_ == nullptr) {
    E("Unable to open camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseAll();
    return -1;
  }

  state_ = State::kOpened;
  D("Camera '%s' is now opened", camera_id_.c_str());
  return 0;
}

int AndroidCameraDevice::getInfo(const std::string& camera_id, CameraInfo* ci) {
  ACameraManager* manager = ACameraManager_create();
  if (manager == nullptr) {
    E("Unable to create camera manager for camera '%s'", camera_id.c_str());
    return -1;
  }

  ACameraMetadata* characteristics = nullptr;
  const camera_status_t status =
      ACameraManager_getCameraCharacteristics(manager, camera_id.c_str(), &characteristics);
  if (status != ACAMERA_OK || characteristics == nullptr) {
    E("Unable to query characteristics of camera '%s': %s",
      camera_id.c_str(), status_to_string(status));
    ACameraManager_delete(manager);
    return -1;
  }

  auto frame_sizes = collect_frame_sizes(characteristics);
  if (frame_sizes->empty()) {
    /* Some devices do not report YUV stream configurations but still capture
     * YUV, fall back to the sizes mandated by the API level. */
    W("Camera '%s' reports no YUV stream configurations, using defaults",
      camera_id.c_str());
    for (const auto& s : kDefaultSizes) {
      CameraFrameDim dim;
      dim.width = s.width;
      dim.height = s.height;
      frame_sizes->push_back(dim);
    }
  }

  /* Direction, used by camera_service to expose front/back facing cameras. */
  int32_t lens_facing = ACAMERA_LENS_FACING_FRONT;
  metadata_get_int32(characteristics, ACAMERA_LENS_FACING, &lens_facing);

  ci->display_name = camera_id;
  ci->device_name = camera_id;
  ci->inp_channel = 0;
  ci->pixel_format = kGuestPixelFormat;
  ci->direction = lens_facing == ACAMERA_LENS_FACING_BACK ? "back" : "front";
  ci->frame_sizes = frame_sizes;
  ci->in_use = false;

  ACameraMetadata_free(characteristics);
  ACameraManager_delete(manager);

  D("Camera '%s' (%s): %u frame size(s)",
    camera_id.c_str(), ci->direction.c_str(),
    static_cast<unsigned>(frame_sizes->size()));
  return 0;
}

int AndroidCameraDevice::startCapturing(uint32_t pixel_format,
                                        unsigned int frame_width,
                                        unsigned int frame_height) {
  if (state_ != State::kOpened) {
    E("Camera '%s' is not opened", camera_id_.c_str());
    return -1;
  }

  /* TODO(step 3): create the AImageReader, the capture session and the
   * repeating capture request here. */
  E("Capturing on camera '%s' is not implemented yet", camera_id_.c_str());
  return -1;
}

int AndroidCameraDevice::stopCapturing() {
  /* TODO(step 3): stop repeating, waitUntilIdle, tear the session down. */
  return 0;
}

int AndroidCameraDevice::readFrame(ClientFrameBuffer* framebuffers,
                                   int fbs_num,
                                   float r_scale,
                                   float g_scale,
                                   float b_scale,
                                   float exp_comp) {
  UNUSED(framebuffers);
  UNUSED(fbs_num);
  UNUSED(r_scale);
  UNUSED(g_scale);
  UNUSED(b_scale);
  UNUSED(exp_comp);

  /* TODO(step 3): convert the latest frame captured by the image reader. */
  E("Reading frames from camera '%s' is not implemented yet", camera_id_.c_str());
  return -1;
}

void AndroidCameraDevice::closeCameraDevice() {
  releaseAll();
}

/*******************************************************************************
 *                     CameraDevice API
 ******************************************************************************/

std::shared_ptr<CameraDevice> CameraDevice::openCameraDevice(const char* name,
                                                             int inp_channel) {
  std::string camera_id;

  if (name != nullptr && *name != '\0') {
    camera_id = name;
  } else {
    /* No explicit camera was requested, pick the first one the manager
     * reports. */
    ACameraManager* manager = ACameraManager_create();
    if (manager == nullptr) {
      E("Unable to create camera manager");
      return nullptr;
    }

    ACameraIdList* id_list = nullptr;
    const camera_status_t status = ACameraManager_getCameraIdList(manager, &id_list);
    if (status != ACAMERA_OK || id_list == nullptr || id_list->numCameras == 0) {
      E("No camera device available: %s", status_to_string(status));
      if (id_list != nullptr) {
        ACameraManager_deleteCameraIdList(id_list);
      }
      ACameraManager_delete(manager);
      return nullptr;
    }

    camera_id = id_list->cameraIds[0];
    ACameraManager_deleteCameraIdList(id_list);
    ACameraManager_delete(manager);
  }

  auto cd = std::make_shared<AndroidCameraDevice>(camera_id, inp_channel);
  if (cd->openDevice()) {
    return nullptr;
  }

  return cd;
}

std::shared_ptr<std::vector<CameraInfo>>
CameraDevice::enumerateCameraDevices(int max) {
  const auto result = std::make_shared<std::vector<CameraInfo>>();

  ACameraManager* manager = ACameraManager_create();
  if (manager == nullptr) {
    E("Unable to create camera manager");
    return result;
  }

  ACameraIdList* id_list = nullptr;
  const camera_status_t status = ACameraManager_getCameraIdList(manager, &id_list);
  if (status != ACAMERA_OK || id_list == nullptr) {
    E("Unable to enumerate cameras: %s", status_to_string(status));
    if (id_list != nullptr) {
      ACameraManager_deleteCameraIdList(id_list);
    }
    ACameraManager_delete(manager);
    return result;
  }

  for (int n = 0; n < max && n < id_list->numCameras; n++) {
    /* The id string is only valid until deleteCameraIdList(), getInfo() copies
     * everything it needs. */
    CameraInfo ci;
    if (AndroidCameraDevice::getInfo(id_list->cameraIds[n], &ci) != 0) {
      W("Skipping camera '%s': unable to collect its capabilities",
        id_list->cameraIds[n]);
      continue;
    }
    result->push_back(ci);
  }

  ACameraManager_deleteCameraIdList(id_list);
  ACameraManager_delete(manager);

  return result;
}

}
