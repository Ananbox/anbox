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
 * Adapted for Anbox by Alec Barber 2021
 */

#ifndef ANDROID_CAMERA_CAMERA_FORMAT_CONVERTERS_H
#define ANDROID_CAMERA_CAMERA_FORMAT_CONVERTERS_H

/*
 * Contains declaration of the API that allows converting frames from one
 * pixel format to another.
 *
 * For the emulator, we really need to convert into two formats: YV12, which is
 * used by the camera framework for video, and RGB32 for preview window.
 */

#include "camera_common.h"

namespace anbox::camera {

/* Checks if conversion between two pixel formats is available.
 * Param:
 *  from - Pixel format to convert from.
 *  to - Pixel format to convert to.
 * Return:
 *  boolean: 1 if converter is available, or 0 if no conversion exists.
 */
int has_converter(uint32_t from, uint32_t to);

/* Converts a frame into multiple framebuffers.
 * When camera service replies to a framebuffer request from the client, it
 * usualy sends two framebuffers in the reply: one for video, and another for
 * preview window. Since these two framebuffers have different pixel formats
 * (most of the time), we need to do two conversions for each frame received from
 * the camera. This is the main intention behind this routine: to have a one call
 * that produces as many conversions as needed.
 * Param:
 *  frame - Frame to convert.
 *  pixel_format - Defines pixel format for the converting framebuffer.
 *  framebuffer_size, width, height - Converting framebuffer byte size, width,
 *      and height.
 *  framebuffers - Array of framebuffers where to convert the frame. Size of this
 *      array is defined by the 'fbs_num' parameter. Note that the caller must
 *      make sure that buffers are large enough to contain entire frame captured
 *      from the device.
 *  fbs_num - Number of entries in the 'framebuffers' array.
 *  r_scale, g_scale, b_scale - White balance scale.
 *  exp_comp - Expsoure compensation.
 * Return:
 *  0 on success, or non-zero value on failure.
 */
int convert_frame(const void* frame,
                  uint32_t pixel_format,
                  size_t framebuffer_size,
                  int width,
                  int height,
                  ClientFrameBuffer* framebuffers,
                  int fbs_num,
                  float r_scale,
                  float g_scale,
                  float b_scale,
                  float exp_comp);

/* Describes one plane of an externally supplied YUV frame.
 *
 * This is used by backends that receive frames from an API which hands out
 * one descriptor per plane and where lines may be padded, e.g. the Android NDK
 * AImage/AHardwareBuffer. Each descriptor points at the first byte of the
 * plane and describes how to walk it.
 *
 * All fields are in bytes.
 */
typedef struct PlaneDesc {
  /* Address of the first byte of the plane. */
  const uint8_t*  data;
  /* Distance between two vertically adjacent pixels of the plane. */
  size_t          row_stride;
  /* Distance between two horizontally adjacent pixels of the plane. For an
   * interleaved chroma plane this is the distance between the two chroma
   * components, i.e. 2. */
  size_t          pixel_stride;
} PlaneDesc;

/* Packs a three plane YUV 4:2:0 frame into a YV12 buffer.
 *
 * The Android NDK reports YUV 4:2:0 frames as one full resolution plane plus
 * two half resolution chroma planes which may be interleaved and may be
 * padded. The converters below expect tightly packed YV12, so the frame has
 * to be repacked first.
 *
 * Param:
 *  y, u, v - Plane descriptors, each pointing at the first byte of its own
 *      plane. U and V may alias one buffer (the NDK case) or live in separate
 *      buffers; only y may be NULL-checked defensively.
 *  width, height - Frame dimensions in pixels.
 *  dst_format - Either V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_YUV420,
 *      V4L2_PIX_FMT_NV12 or V4L2_PIX_FMT_NV21.
 *  dst - Destination buffer.
 *  dst_size - Size of the destination buffer. Must be at least
 *      width * height * 3 / 2.
 * Return:
 *  0 on success, non-zero on failure.
 */
int pack_yuv420_planes(const PlaneDesc& y,
                       const PlaneDesc& u,
                       const PlaneDesc& v,
                       int width,
                       int height,
                       uint32_t dst_format,
                       uint8_t* dst,
                       size_t dst_size);

}

#endif  /* ANDROID_CAMERA_CAMERA_FORMAT_CONVERTERS_H */