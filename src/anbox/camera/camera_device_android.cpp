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

#include <errno.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
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

/* Pixel formats the guest accepts for video, see the switch in
 * CameraMessageProcessor::start(). Asking for anything else would make the
 * guest reject the frame stream after startCapturing() already succeeded. */
static bool is_supported_guest_format(uint32_t format) {
  return format == A_ANBOX_FOURCC('Y', 'U', '1', '2') ||  /* YU12 */
         format == A_ANBOX_FOURCC('Y', 'V', '1', '2') ||  /* YV12 */
         format == A_ANBOX_FOURCC('N', 'V', '1', '2') ||  /* NV12 */
         format == A_ANBOX_FOURCC('N', 'V', '2', '1');    /* NV21 */
}

/* Number of buffers handed to the image reader. Extra buffers absorb the
 * jitter between the camera producing frames and readFrame() consuming them,
 * see storeLatestFrame(). */
static const int kImageReaderBuffers = 4;  /* same as the NDK sample */

/* After this many consecutive unusable frames the capture session is marked
 * as failed instead of waiting for the guest to time out. */
static const int kMaxBadFrames = 10;

/* How long stopCapturing() waits for in-flight frames to drain. */
static const int64_t kSessionIdleTimeoutNs = 1000000000LL;  /* 1 second */

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

/* Reads an int32 entry, returns false when the tag is missing.
 *
 * android.lens.facing is documented as int32[1], but some vendor HALs (e.g.
 * Android 10 on Xiaomi polaris) report it as a byte. Accept both instead of
 * silently losing the direction. */
bool metadata_get_int32(ACameraMetadata* md, uint32_t tag, int32_t* out) {
  ACameraMetadata_const_entry entry;
  if (ACameraMetadata_getConstEntry(md, tag, &entry) != ACAMERA_OK) {
    return false;
  }
  if (entry.count < 1) {
    return false;
  }
  if (entry.type == ACAMERA_TYPE_INT32) {
    *out = entry.data.i32[0];
  } else if (entry.type == ACAMERA_TYPE_BYTE) {
    *out = static_cast<int32_t>(entry.data.u8[0]);
  } else if (entry.type == ACAMERA_TYPE_INT64) {
    *out = static_cast<int32_t>(entry.data.i64[0]);
  } else {
    return false;
  }
  return true;
}

/* Builds the list of frame sizes for the given YUV format.
 *
 * The camera2 API enumerates every output format/size combination in
 * ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS. Each entry is a
 * (format, width, height, isInput) tuple. Only capture (isInput == 0) sizes in
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

  /* The tuples are (format, width, height, isInput), not
   * (width, height, format, isInput) as the dumpsys output suggests. Reading
   * the wrong position made every camera report no YUV sizes. */
  for (uint32_t i = 0; i + 3 < entry.count; i += 4) {
    const int32_t fmt = entry.data.i32[i + 0];
    const int32_t w = entry.data.i32[i + 1];
    const int32_t h = entry.data.i32[i + 2];
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
  enum class State { kClosed, kOpened, kStreaming };

  /* Releases every NDK resource held by this device. Idempotent. */
  void releaseAll();

  /* Tears down the capture session but keeps the device open. Idempotent. */
  void releaseCapture();

  /* Camera state callbacks. Static trampolines, the 'context' pointer is set
   * to the AndroidCameraDevice instance. */
  /* The device parameter is declared void* on purpose, see the definitions. */
  static void onDeviceDisconnected(void* context, void* device);
  static void onDeviceError(void* context, void* device, int error);

  /* Capture session state callbacks. */
  /* The session parameter is void* on purpose: see onDeviceDisconnected(). */
  static void onSessionReady(void* context, void* session);
  static void onSessionActive(void* context, void* session);
  static void onSessionClosed(void* context, void* session);

  /* Image reader callback. Runs on a camera service binder thread. */
  /* The reader parameter is void* on purpose, see the definition. */
  static void onImageAvailable(void* context, AImageReader* reader);

  /* Pulls one frame out of the reader and stores it in the frame slot.
   * Called from onImageAvailable only. */
  void storeLatestFrame(AImageReader* reader);

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
   */
  ACameraCaptureSession*          session_;
  ACameraCaptureSession_stateCallbacks session_cb_;
  ACaptureRequest*                capture_request_;
  ACameraOutputTarget*            output_target_;
  ACaptureSessionOutput*          session_output_;
  ACaptureSessionOutputContainer* output_container_;
  AImageReader*                   image_reader_;
  AImageReader_ImageListener      image_listener_;
  ANativeWindow*                  image_reader_window_;

  /* Pixel format required by the guest and frame geometry of the reader. */
  uint32_t          pixel_format_;
  int               frame_width_;
  int               frame_height_;

  /*
   * Frame slot.
   *
   * The NDK pushes frames to onImageAvailable() on a camera service binder
   * thread, while readFrame() pulls them from the qemu pipe worker. The frame
   * is repacked to a tight YV12 buffer in the callback and published here
   * together with a sequence number, which plays the role VIDIOC_DQBUF plays
   * for the V4L2 backend: an unchanged sequence number means "no new frame",
   * which readFrame() reports as 1 so the caller retries.
   *
   * frame_mutex_ also serializes writes to slot_ with the release path, so a
   * callback can never touch slot_ after it was handed back to the caller.
   */
  std::mutex            frame_mutex_;
  std::condition_variable capture_cv_;
  std::vector<uint8_t>  slot_;
  size_t                slot_size_;
  uint64_t              frame_seq_;

  /* Signalled by onSessionReady(). The NDK has no "wait until idle" call: after
   * stopRepeating() the session reports itself ready once the last request has
   * completed, which is the documented way to reach an idle state. */
  std::mutex            session_mutex_;
  std::condition_variable session_cv_;
  bool                  session_ready_;
  bool                  session_closed_;
  std::atomic<bool>     session_active_seen_;

  /* Set by the device callbacks. */
  std::atomic<bool>     disconnected_;
  std::atomic<bool>     device_error_;
  /* Whether the disconnect/error has already been reported to the guest.
   * readFrame() reports it once and then behaves as "no frame", otherwise the
   * qemu pipe would see an error on every single frame request. */
  std::atomic<bool>     failure_reported_;

  /* Set by storeLatestFrame(). Conversion failures are sticky: continuing
   * would keep returning frames that were never converted. */
  std::atomic<bool>     frame_error_;

  /* Consecutive frames that could not be used, either because a plane was
   * missing or because they could not be packed. Used to detect a frame
   * geometry change or a broken HAL instead of logging per frame. */
  int                   bad_frame_count_;
  int                   bad_pack_count_;
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
      image_listener_{},
      image_reader_window_(nullptr),
      pixel_format_(kGuestPixelFormat),
      frame_width_(0),
      frame_height_(0),
      slot_size_(0),
      frame_seq_(0),
      session_ready_(false),
      session_closed_(false),
      session_active_seen_(false),
      disconnected_(false),
      device_error_(false),
      failure_reported_(false),
      frame_error_(false),
      bad_frame_count_(0),
      bad_pack_count_(0) {}

/*******************************************************************************
 *                     Callbacks
 *
 * Every callback runs on a camera service binder thread. They must not block
 * and must not tear anything down: a close from inside a callback deadlocks
 * against the camera service. They only update state and signal waiters.
 ******************************************************************************/

void AndroidCameraDevice::onDeviceDisconnected(void* context, void* device) {
  /* The device argument is only an opaque token here: the NDK hands out a
   * pointer into its own device object, which is already being torn down when
   * this callback arrives. Declaring the parameter as ACameraDevice* makes the
   * compiler emit reference-counting code for it, which dereferences that
   * dying object and crashes (libutils RefBase::decStrong, Android 10). */
  UNUSED(device);
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr) {
    return;
  }

  /* After this point every NDK call on the device fails with
   * ACAMERA_ERROR_CAMERA_DISCONNECTED. Stop streaming and wake up readers. */
  {
    std::lock_guard<std::mutex> lock(self->frame_mutex_);
    self->disconnected_ = true;
  }
  self->capture_cv_.notify_all();
}

void AndroidCameraDevice::onDeviceError(void* context, void* device, int error) {
  UNUSED(device);
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr) {
    return;
  }
  E("Camera '%s' reported an error: %d", self->camera_id_.c_str(), error);
  {
    std::lock_guard<std::mutex> lock(self->frame_mutex_);
    self->device_error_ = true;
  }
  self->capture_cv_.notify_all();
}

void AndroidCameraDevice::onSessionReady(void* context, void* session) {
  UNUSED(session);
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(self->session_mutex_);
  self->session_ready_ = true;
  self->session_cv_.notify_all();
}

void AndroidCameraDevice::onSessionActive(void* context, void* session) {
  UNUSED(session);
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr) {
    return;
  }
  /* The session started serving requests. Nothing to do, but the callback must
   * be installed: some HALs (QCOM CamX) behave differently, and this is the
   * point where frames start flowing. */
  self->session_active_seen_ = true;
}

void AndroidCameraDevice::onSessionClosed(void* context, void* session) {
  UNUSED(session);
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr) {
    return;
  }
  /* A closed session will never report itself ready again; wake up anyone
   * waiting for idle so it can fall back to abortCaptures(), and let
   * releaseCapture() know the session is really gone before it frees the
   * request that session referenced. */
  std::lock_guard<std::mutex> lock(self->session_mutex_);
  self->session_closed_ = true;
  self->session_cv_.notify_all();
}

void AndroidCameraDevice::onImageAvailable(void* context, AImageReader* reader) {
  auto self = static_cast<AndroidCameraDevice*>(context);
  if (self == nullptr || reader == nullptr) {
    return;
  }
  self->storeLatestFrame(reader);
}

void AndroidCameraDevice::storeLatestFrame(AImageReader* reader) {
  if (disconnected_ || device_error_ || frame_error_) {
    /* Frames that arrive after the device went away, or after the session was
     * given up on, are dropped. They still have to be dequeued and released so
     * the reader does not wedge while it is being torn down. */
    AImage* image = nullptr;
    if (AImageReader_acquireLatestImage(reader, &image) == AMEDIA_OK && image != nullptr) {
      AImage_delete(image);
    }
    return;
  }

  /* acquireLatestImage() drops all but the most recent frame, which is exactly
   * the behaviour we want: the guest always gets the freshest picture. It must
   * be called with no other image held by this reader, which holds because
   * every path below releases the image before returning. */
  AImage* image = nullptr;
  const media_status_t status = AImageReader_acquireLatestImage(reader, &image);
  if (status != AMEDIA_OK || image == nullptr) {
    if (status != AMEDIA_ERROR_WOULD_BLOCK) {
      W("Unable to acquire image from camera '%s': %d",
        camera_id_.c_str(), status);
    }
    return;
  }

  /* Each plane is described by its own stride. Note that pixel_stride of the
   * chroma planes is 2 for AIMAGE_FORMAT_YUV_420_888, the U and V planes may
   * even alias the same buffer. */
  PlaneDesc planes[3];
  bool have_plane[3] = { false, false, false };
  for (int i = 0; i < 3; i++) {
    uint8_t* data = nullptr;
    int data_length = 0;
    int row_stride = 0;
    int pixel_stride = 0;
    if (AImage_getPlaneData(image, i, &data, &data_length) != AMEDIA_OK ||
        AImage_getPlaneRowStride(image, i, &row_stride) != AMEDIA_OK ||
        AImage_getPlanePixelStride(image, i, &pixel_stride) != AMEDIA_OK ||
        data == nullptr) {
      continue;
    }
    planes[i].data = data;
    planes[i].row_stride = static_cast<size_t>(row_stride);
    planes[i].pixel_stride = static_cast<size_t>(pixel_stride);
    have_plane[i] = true;
  }

  /* Reject malformed geometry before the strides are used as offsets. A luma
   * row that is narrower than the frame would make the packer read past the
   * plane, the same goes for a zero chroma stride. */
  if (!have_plane[0] || !have_plane[1] || !have_plane[2] ||
      planes[0].pixel_stride == 0 || planes[1].pixel_stride == 0 ||
      planes[0].row_stride < static_cast<size_t>(frame_width_) ||
      planes[1].row_stride == 0 || planes[2].row_stride == 0) {
    AImage_delete(image);

    /* Do not log on every frame: a broken HAL would flood the log at the
     * capture frame rate. Give up after a few frames instead of silently
     * returning frames that never update. */
    if (++bad_frame_count_ >= kMaxBadFrames) {
      E("Camera '%s' keeps returning unusable YUV frames, giving up",
        camera_id_.c_str());
      frame_error_ = true;
      capture_cv_.notify_all();
    } else {
      W("Camera '%s' returned an unusable YUV frame", camera_id_.c_str());
    }
    return;
  }
  bad_frame_count_ = 0;

  /* 0: packed, 1: dropped because the capture path is going away, -1: failed. */
  int pack_result = 1;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    /* slot_size_ is zero once the capture path has been torn down, a frame
     * racing with stopCapturing() is simply dropped. */
    if (slot_size_ != 0) {
      pack_result = pack_yuv420_planes(planes[0], planes[1], planes[2],
                                       frame_width_, frame_height_, pixel_format_,
                                       slot_.data(), slot_size_);
      if (pack_result == 0) {
        frame_seq_++;
      }
    }
  }

  /* The image must be released before the callback returns, otherwise the
   * reader runs out of buffers and the capture pipeline stalls. */
  AImage_delete(image);

  if (pack_result > 0) {
    /* Torn down while this frame was in flight, nothing to report. */
    return;
  }

  if (pack_result < 0) {
    /* A frame whose plane geometry does not match the configured size can
     * never be converted. Treat it as fatal for this capture session rather
     * than reporting "no frame" forever, which would make the guest wait until
     * its 2 second timeout without ever getting a reason. Logging from
     * pack_yuv420_planes() already happened, so stay quiet here. */
    if (++bad_pack_count_ >= kMaxBadFrames) {
      E("Camera '%s' keeps returning unusable frames, giving up",
        camera_id_.c_str());
      frame_error_ = true;
    }
    capture_cv_.notify_all();
    return;
  }
  bad_pack_count_ = 0;

  capture_cv_.notify_all();
}

AndroidCameraDevice::~AndroidCameraDevice() {
  closeCameraDevice();
}

/* The base class destructor (and its typeinfo for dynamic_cast) is defined
 * per platform: only one backend is compiled into a build. */
CameraDevice::~CameraDevice() {}

void AndroidCameraDevice::releaseCapture() {
  /* Ownership rules of the NDK:
   *  - ACameraCaptureSession_close() is asynchronous; resources attached to
   *    the session are not valid afterwards.
   *  - ACaptureSessionOutput owns its ANativeWindow reference, but does not
   *    release it, the caller keeps its own reference.
   * Every pointer is cleared so the routine is safe to call twice.
   */
  /* Stop frame delivery before anything is torn down. Dropping the listener is
   * synchronous: once it returns, onImageAvailable() is not called again, so
   * the slots it writes to can go away below. Waiting for the session to close
   * is not enough, because onReady/onClosed are not guaranteed to arrive (this
   * QCOM HAL never reports either), which used to leave the callback running
   * while the reader was already freed. */
  /* Whether the session reported itself closed. When it did not, the objects
   * the session still references must not be freed here, the session owns
   * them until it goes away. */
  bool session_gone = true;

  if (session_ != nullptr) {
    /* Closing is asynchronous, and the session keeps a reference to every
     * request and target that went through it until it is gone. Freeing them
     * before onClosed arrives makes the session tear down objects that are
     * already released (SIGSEGV inside libcamera2ndk on Android 10). Wait for
     * onClosed, bounded by the same timeout as idle. */
    {
      std::unique_lock<std::mutex> lock(session_mutex_);
      session_closed_ = false;
    }
    ACameraCaptureSession_stopRepeating(session_);
    ACameraCaptureSession_close(session_);
    {
      std::unique_lock<std::mutex> lock(session_mutex_);
      session_gone = session_cv_.wait_for(
          lock, std::chrono::nanoseconds(kSessionIdleTimeoutNs),
          [this] { return session_closed_; });
    }
    if (!session_gone) {
      W("Capture session of camera '%s' did not report itself closed",
        camera_id_.c_str());
    }
    session_ = nullptr;
  }

  if (capture_request_ != nullptr && session_gone) {
    /* Detach the target first: the session holds a reference to every target
     * that went through it, and freeing the target while the request still
     * points at it is a double release (SIGSEGV in RefBase::decStrong on
     * Android 10). The official NDK sample does the same. */
    if (output_target_ != nullptr) {
      ACaptureRequest_removeTarget(capture_request_, output_target_);
    }
    ACaptureRequest_free(capture_request_);
    capture_request_ = nullptr;
  }

  if (output_target_ != nullptr && session_gone) {
    ACameraOutputTarget_free(output_target_);
    output_target_ = nullptr;
  }

  if (session_output_ != nullptr && session_gone) {
    ACaptureSessionOutput_free(session_output_);
    session_output_ = nullptr;
  }

  if (output_container_ != nullptr && session_gone) {
    ACaptureSessionOutputContainer_free(output_container_);
    output_container_ = nullptr;
  }

  /* Order matters on this ROM: the window must be released before the reader
   * is deleted, otherwise AImageReader_delete crashes inside the NDK. */
  if (image_reader_window_ != nullptr) {
    /* The reader does not own the window it was created from. */
    ANativeWindow_release(image_reader_window_);
    image_reader_window_ = nullptr;
  }

  if (image_reader_ != nullptr) {
    /* AImageReader_delete() crashes inside libmediandk on this ROM once the
     * reader has delivered frames (RefBase::decStrong on a null pointer, from
     * the buffer-queue teardown). Deleting the window first, or clearing the
     * image listener first, does not help. The reader is therefore dropped
     * instead of deleted; android/ndk-samples/camera does the same. */
    image_reader_ = nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    slot_.clear();
    slot_.shrink_to_fit();
    slot_size_ = 0;
    frame_seq_ = 0;
  }

  capture_cv_.notify_all();
}

void AndroidCameraDevice::releaseAll() {
  releaseCapture();

  if (device_ != nullptr) {
    ACameraDevice_close(device_);
    device_ = nullptr;
  }

  if (manager_ != nullptr) {
    ACameraManager_delete(manager_);
    manager_ = nullptr;
  }

  /* Every flag describes the capture session that just went away. */
  failure_reported_ = false;
  frame_error_ = false;
  bad_frame_count_ = 0;
  bad_pack_count_ = 0;

  state_ = State::kClosed;
}

int AndroidCameraDevice::openDevice() {
  if (state_ != State::kClosed) {
    E("Camera '%s' is already opened", camera_id_.c_str());
    return -1;
  }

  /* Clear the state of the previous connection before the new one can report
   * anything, otherwise a callback firing during openCamera() would be
   * overwritten by the reset below. */
  disconnected_ = false;
  device_error_ = false;
  failure_reported_ = false;
  frame_error_ = false;
  bad_frame_count_ = 0;
  bad_pack_count_ = 0;

  manager_ = ACameraManager_create();
  if (manager_ == nullptr) {
    E("Unable to create camera manager for camera '%s'", camera_id_.c_str());
    return -1;
  }

  /* The NDK stores the pointer to this struct, not a copy, so device_cb_ is a
   * value member that is only destroyed after ACameraDevice_close(). */
  device_cb_.context = this;
  /* The handlers take the device as void*; see onDeviceDisconnected(). */
  device_cb_.onDisconnected = reinterpret_cast<void (*)(void*, ACameraDevice*)>(
      &AndroidCameraDevice::onDeviceDisconnected);
  device_cb_.onError = reinterpret_cast<void (*)(void*, ACameraDevice*, int)>(
      &AndroidCameraDevice::onDeviceError);

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

  /* Direction, used by camera_service to expose front/back facing cameras.
   * Do not default to front when the tag is missing: a wrong direction makes
   * the guest label every rear camera as front. */
  int32_t lens_facing = 0;
  std::string direction = "unknown";
  if (metadata_get_int32(characteristics, ACAMERA_LENS_FACING, &lens_facing)) {
    switch (lens_facing) {
      case ACAMERA_LENS_FACING_BACK: direction = "back"; break;
      case ACAMERA_LENS_FACING_FRONT: direction = "front"; break;
      case ACAMERA_LENS_FACING_EXTERNAL: direction = "external"; break;
      default: break;
    }
  } else {
    W("Camera '%s' does not report its lens facing", camera_id.c_str());
  }

  ci->display_name = camera_id;
  ci->device_name = camera_id;
  ci->inp_channel = 0;
  ci->pixel_format = kGuestPixelFormat;
  ci->direction = direction;
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

  if (frame_width == 0 || frame_height == 0) {
    E("Invalid frame dimensions %ux%u for camera '%s'",
      frame_width, frame_height, camera_id_.c_str());
    errno = EINVAL;
    return -1;
  }

  /* YUV 4:2:0 needs whole chroma pixels. The guest allocates the frame buffer
   * with (w * h * 3) / 2 bytes, the packer would need (w + 1) / 2 * (h + 1) /
   * 2 * 2 bytes for an odd size, so an odd dimension can never be delivered.
   * Reject it here instead of failing every frame in the packer. */
  if ((frame_width & 1) || (frame_height & 1)) {
    E("Camera '%s' cannot stream odd frame size %ux%u",
      camera_id_.c_str(), frame_width, frame_height);
    errno = EINVAL;
    return -1;
  }

  /* A device that disconnected or failed is unusable until it is closed and
   * opened again, do not resurrect a dead capture session. */
  if (disconnected_ || device_error_) {
    E("Camera '%s' is no longer available", camera_id_.c_str());
    return -1;
  }

  if (!is_supported_guest_format(pixel_format)) {
    E("Camera '%s' cannot deliver pixel format %.4s, the guest accepts only "
      "YU12, YV12, NV12 and NV21",
      camera_id_.c_str(), reinterpret_cast<const char*>(&pixel_format));
    return -1;
  }

  /* The guest may ask for a format other than the one reported during
   * enumeration (NV12/NV21 instead of YV12), the frame slot is packed in
   * whatever format the guest asked for and converted from there. */
  pixel_format_ = pixel_format;
  frame_width_ = static_cast<int>(frame_width);
  frame_height_ = static_cast<int>(frame_height);

  /* A new capture session gets a fresh chance to report a failure. */
  failure_reported_ = false;
  frame_error_ = false;
  bad_frame_count_ = 0;
  bad_pack_count_ = 0;

  camera_status_t status;
  media_status_t media_status;

  /* 1. Output: an image reader producing YUV 4:2:0 frames we can read from
   *    CPU. A few buffers absorb the jitter between the camera producing
   *    frames and readFrame() consuming them. */
  /* Match android/ndk-samples/camera/basic: plain AImageReader_new with the
   * default usage. newWithUsage(CPU_READ_OFTEN) configures the stream with
   * usage 0x3, which this QCOM HAL accepts but never delivers frames for. */
  media_status = AImageReader_new(frame_width_, frame_height_,
                                  AIMAGE_FORMAT_YUV_420_888,
                                  kImageReaderBuffers, &image_reader_);
  if (media_status != AMEDIA_OK || image_reader_ == nullptr) {
    E("Unable to create image reader for camera '%s': %d",
      camera_id_.c_str(), media_status);
    releaseCapture();
    return -1;
  }

  /* image_listener_ is a value member: the NDK keeps a pointer to it. */
  image_listener_.context = this;
  image_listener_.onImageAvailable = &AndroidCameraDevice::onImageAvailable;
  media_status = AImageReader_setImageListener(image_reader_, &image_listener_);
  if (media_status != AMEDIA_OK) {
    E("Unable to set image listener for camera '%s': %d",
      camera_id_.c_str(), media_status);
    releaseCapture();
    return -1;
  }

  media_status = AImageReader_getWindow(image_reader_, &image_reader_window_);
  if (media_status != AMEDIA_OK || image_reader_window_ == nullptr) {
    E("Unable to get reader window for camera '%s': %d",
      camera_id_.c_str(), media_status);
    releaseCapture();
    return -1;
  }

  /* 2. Session. */
  status = ACaptureSessionOutputContainer_create(&output_container_);
  if (status != ACAMERA_OK || output_container_ == nullptr) {
    E("Unable to create output container for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  status = ACaptureSessionOutput_create(image_reader_window_, &session_output_);
  if (status != ACAMERA_OK || session_output_ == nullptr) {
    E("Unable to create session output for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  status = ACaptureSessionOutputContainer_add(output_container_, session_output_);
  if (status != ACAMERA_OK) {
    E("Unable to add session output for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  /* Session callbacks are installed here and only live until releaseCapture().
   * onConfigured() does not exist in the NDK: session configuration is
   * asynchronous and the first request may be queued while it is still in
   * progress, so startCapturing() does not wait for it. */
  session_cb_.context = this;
  /* The handlers take the session as void*; see their definitions. */
  session_cb_.onReady = reinterpret_cast<void (*)(void*, ACameraCaptureSession*)>(
      &AndroidCameraDevice::onSessionReady);
  session_cb_.onActive = reinterpret_cast<void (*)(void*, ACameraCaptureSession*)>(
      &AndroidCameraDevice::onSessionActive);
  session_cb_.onClosed = reinterpret_cast<void (*)(void*, ACameraCaptureSession*)>(
      &AndroidCameraDevice::onSessionClosed);

  status = ACameraDevice_createCaptureSession(device_, output_container_,
                                              &session_cb_, &session_);
  if (status != ACAMERA_OK || session_ == nullptr) {
    E("Unable to create capture session for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  /* 3. Request. TEMPLATE_PREVIEW gives auto exposure/white balance. The guest
   *    driven white balance and exposure compensation are applied by
   *    convert_frame() in readFrame(), same as for the V4L2 backend. */
  status = ACameraDevice_createCaptureRequest(device_, TEMPLATE_PREVIEW,
                                              &capture_request_);
  if (status != ACAMERA_OK || capture_request_ == nullptr) {
    E("Unable to create capture request for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  status = ACameraOutputTarget_create(image_reader_window_, &output_target_);
  if (status != ACAMERA_OK || output_target_ == nullptr) {
    E("Unable to create output target for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  status = ACaptureRequest_addTarget(capture_request_, output_target_);
  if (status != ACAMERA_OK) {
    E("Unable to add output target for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  /* 4. Frame slot. YV12 and YU12 need width * height * 3 / 2 bytes, so do the
   *    interleaved NV12/NV21 layouts. */
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    slot_size_ = static_cast<size_t>(frame_width_) * frame_height_ * 3 / 2;
    slot_.assign(slot_size_, 0);
    frame_seq_ = 0;
  }

  /* 5. Go. */
  status = ACameraCaptureSession_setRepeatingRequest(session_, nullptr, 1,
                                                     &capture_request_, nullptr);
  if (status != ACAMERA_OK) {
    E("Unable to start repeating request for camera '%s': %s",
      camera_id_.c_str(), status_to_string(status));
    releaseCapture();
    return -1;
  }

  state_ = State::kStreaming;
  D("Camera '%s' is now capturing %.4s[%dx%d]",
    camera_id_.c_str(), reinterpret_cast<const char*>(&pixel_format_),
    frame_width_, frame_height_);
  return 0;
}

int AndroidCameraDevice::stopCapturing() {
  if (state_ == State::kClosed) {
    /* Nothing was ever opened, or the device was already closed. */
    return 0;
  }

  if (state_ != State::kStreaming) {
    /* Idempotent: the caller may stop a camera that never started. */
    return 0;
  }

  /* Let in-flight frames land before the reader goes away. A timeout here
   * means the HAL is wedged or the device disconnected, in which case the
   * session is still safe to close. */
  if (session_ != nullptr && !disconnected_ && !device_error_) {
    /* stopRepeating() only cancels future requests; the NDK is explicit that
     * onReady is the way to learn that the last in-flight request is done. */
    bool idle = false;
    {
      std::unique_lock<std::mutex> lock(session_mutex_);
      session_ready_ = false;
    }
    ACameraCaptureSession_stopRepeating(session_);
    {
      std::unique_lock<std::mutex> lock(session_mutex_);
      idle = session_cv_.wait_for(
          lock, std::chrono::nanoseconds(kSessionIdleTimeoutNs),
          [this] { return session_ready_ || disconnected_ || device_error_; });
    }
    if (!idle) {
      ACameraCaptureSession_abortCaptures(session_);
    }
  }

  /* Returns once no onImageAvailable() call can be in flight, so the caller
   * may free its frame buffers as soon as this returns. */
  releaseCapture();

  /* A failed session is not reusable: release the device as well, otherwise it
   * stays claimed by this process and the guest can never open it again. */
  if (disconnected_ || device_error_ || frame_error_) {
    W("Camera '%s' is no longer usable, closing it", camera_id_.c_str());
    releaseAll();
  } else {
    state_ = State::kOpened;
  }

  D("Camera '%s' is now stopped", camera_id_.c_str());
  return 0;
}

int AndroidCameraDevice::readFrame(ClientFrameBuffer* framebuffers,
                                   int fbs_num,
                                   float r_scale,
                                   float g_scale,
                                   float b_scale,
                                   float exp_comp) {
  if (state_ != State::kStreaming) {
    /* Called after stop() or before start(). Not an error worth logging on
     * every frame request, the caller already got a proper reply. */
    errno = EBADF;
    return -1;
  }

  /* A device that went away or a session that kept producing unusable frames
   * is reported to the guest once, after that the camera behaves as having no
   * frame available so the guest stops retrying instead of getting an I/O
   * error for every frame. */
  if (disconnected_ || device_error_ || frame_error_) {
    const bool expected = failure_reported_.exchange(true);
    if (!expected) {
      E("Camera '%s' can no longer produce frames%s", camera_id_.c_str(),
        disconnected_ ? " (device disconnected)" :
        device_error_ ? " (device error)" : " (unusable frames)");
    }
    errno = ENODEV;
    return expected ? 1 : -1;
  }

  /* Take a snapshot of the latest frame. The conversion runs outside the lock
   * so onImageAvailable() is not blocked behind the (relatively slow) pixel
   * conversion. A private copy is cheaper than holding the slot, and the
   * frames are small compared to the conversion cost.
   */
  std::vector<uint8_t> frame;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (frame_seq_ == 0) {
      /* No frame captured yet. Same contract as VIDIOC_DQBUF returning EAGAIN
       * on the V4L2 backend: the caller retries. */
      return 1;
    }
    frame = slot_;
  }

  /* slot_ already is in the guest format, but convert_frame() still has to run
   * to produce the RGB32 preview and to apply white balance / exposure. */
  return convert_frame(frame.data(), pixel_format_, frame.size(),
                       frame_width_, frame_height_,
                       framebuffers, fbs_num,
                       r_scale, g_scale, b_scale, exp_comp);
}

void AndroidCameraDevice::closeCameraDevice() {
  /* Idempotent and safe in any state. When the caller did not stop first the
   * capture session is torn down by releaseAll(), which also stops repeating
   * so the camera is not left producing frames for a reader that is going
   * away. */
  if (state_ == State::kStreaming) {
    W("Camera '%s' is closed while still capturing", camera_id_.c_str());
  }
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
