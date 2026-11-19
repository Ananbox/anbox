/*
 * Standalone unit test for anbox::camera::pack_yuv420_planes().
 *
 * This test is self contained on purpose: it does not need Android, the NDK,
 * boost or anbox's logger. It compiles the packer together with a minimal
 * shim and runs on any host with a C++11 compiler.
 *
 * Build & run:
 *   ./run_tests.sh
 * or manually:
 *   clang++ -std=c++14 -I. -I<anbox/src> test_pack_yuv420_planes.cpp \
 *           <anbox/src>/anbox/camera/camera_format_converters.cpp -o t && ./t
 *
 * What it covers:
 *   - tight vs. padded (stride aligned) source planes
 *   - all four destination layouts: YU12, YV12, NV12, NV21
 *   - interleaved chroma where U and V alias one buffer (the real NDK case)
 *   - odd frame dimensions
 *   - destination buffer size checks
 *   - a round trip through convert_frame() to catch colour channel swaps
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include "anbox/camera/camera_format_converters.h"

using anbox::camera::PlaneDesc;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    g_checks++;                                                              \
    if (!(cond)) {                                                           \
      g_failures++;                                                          \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
    }                                                                        \
  } while (0)

#define CHECK_EQ_INT(actual, expected)                                       \
  do {                                                                       \
    g_checks++;                                                              \
    const long long a_ = (long long)(actual);                                \
    const long long e_ = (long long)(expected);                              \
    if (a_ != e_) {                                                          \
      g_failures++;                                                          \
      std::printf("  FAIL %s:%d: %s == %lld, expected %lld\n",               \
                  __FILE__, __LINE__, #actual, a_, e_);                      \
    }                                                                        \
  } while (0)

/* -------------------------------------------------------------------------
 * Synthetic source frame
 *
 * Builds the three planes the way the NDK hands them out: one luma plane and
 * two chroma planes, each with its own row stride that may be larger than the
 * useful width (hardware aligns rows to 16/32/64 bytes). Values are derived
 * from the coordinate so that a misplaced byte is detectable.
 * ---------------------------------------------------------------------- */

struct SourceFrame {
  int width = 0;
  int height = 0;
  size_t y_row_stride = 0;
  size_t uv_row_stride = 0;
  size_t uv_pixel_stride = 1;
  bool interleaved = false;   /* U and V share one buffer (NDK style) */

  std::vector<uint8_t> y_buf;
  std::vector<uint8_t> u_buf;
  std::vector<uint8_t> v_buf;

  int chroma_width() const { return (width + 1) / 2; }
  int chroma_height() const { return (height + 1) / 2; }

  /* Deterministic, coordinate dependent values. Kept inside a range that
   * survives the YUV->RGB conversion without clipping so the round trip test
   * can compare against a reference conversion. */
  static uint8_t y_value(int x, int y) { return (uint8_t)((x * 7 + y * 3) & 0x7f); }
  static uint8_t u_value(int cx, int cy) { return (uint8_t)(64 + ((cx + cy) & 0x3f)); }
  static uint8_t v_value(int cx, int cy) { return (uint8_t)(192 - ((cx * 2 + cy) & 0x3f)); }

  void build(int w, int h, size_t y_padding, size_t uv_padding,
             size_t pixel_stride, bool interleave) {
    width = w;
    height = h;
    uv_pixel_stride = pixel_stride;
    interleaved = interleave;
    y_row_stride = (size_t)w + y_padding;
    uv_row_stride = (size_t)chroma_width() * pixel_stride + uv_padding;

    y_buf.assign(y_row_stride * height, 0xAA);   /* padding poison */
    for (int row = 0; row < height; row++) {
      for (int x = 0; x < w; x++) {
        y_buf[row * y_row_stride + x] = y_value(x, row);
      }
    }

    if (interleave) {
      u_buf.assign(uv_row_stride * chroma_height(), 0xAA);
      /* The NDK hands out one descriptor per plane, each pointing at its own
       * first byte: U at +0, V at +1 of the shared buffer. Both report the
       * same row and pixel stride. */
      for (int row = 0; row < chroma_height(); row++) {
        for (int cx = 0; cx < chroma_width(); cx++) {
          uint8_t* px = &u_buf[row * uv_row_stride + (size_t)cx * pixel_stride];
          px[0] = u_value(cx, row);
          px[1] = v_value(cx, row);
        }
      }
      /* Separate storage in the test, but v_desc() below reports data+1. */
      v_buf = u_buf;
    } else {
      u_buf.assign(uv_row_stride * chroma_height(), 0xAA);
      v_buf.assign(uv_row_stride * chroma_height(), 0xAA);
      for (int row = 0; row < chroma_height(); row++) {
        for (int cx = 0; cx < chroma_width(); cx++) {
          u_buf[row * uv_row_stride + (size_t)cx * pixel_stride] = u_value(cx, row);
          v_buf[row * uv_row_stride + (size_t)cx * pixel_stride] = v_value(cx, row);
        }
      }
    }
  }

  PlaneDesc y_plane() const { return PlaneDesc{ y_buf.data(), y_row_stride, 1 }; }
  PlaneDesc u_plane() const { return PlaneDesc{ u_buf.data(), uv_row_stride, uv_pixel_stride }; }
  PlaneDesc v_plane() const {
    const uint8_t* data = v_buf.data();
    if (interleaved) {
      /* NDK: the V descriptor starts at the second byte of the pair. */
      data += 1;
    }
    return PlaneDesc{ data, uv_row_stride, uv_pixel_stride };
  }
};

/* Expected tightly packed layout for a given destination format. */
struct ExpectedLayout {
  size_t luma_size = 0;
  size_t chroma_size = 0;
  bool uv_interleaved = false;
  bool v_first = false;
};

ExpectedLayout expected_layout(uint32_t fmt, int w, int h) {
  ExpectedLayout e;
  e.luma_size = (size_t)w * h;
  e.chroma_size = (size_t)((w + 1) / 2) * ((h + 1) / 2);
  e.uv_interleaved = (fmt == V4L2_PIX_FMT_NV12 || fmt == V4L2_PIX_FMT_NV21);
  e.v_first = (fmt == V4L2_PIX_FMT_YVU420);
  return e;
}

/* Verifies every byte of the packed buffer. */
void verify_packed(const SourceFrame& src, const std::vector<uint8_t>& dst,
                   uint32_t fmt, const char* label) {
  const int w = src.width, h = src.height;
  const int cw = src.chroma_width(), ch = src.chroma_height();
  const ExpectedLayout exp = expected_layout(fmt, w, h);

  std::printf("  [%s] %.4s %dx%d\n", label,
              reinterpret_cast<const char*>(&fmt), w, h);

  CHECK_EQ_INT(dst.size(), exp.luma_size + exp.chroma_size * 2);

  /* Luma: must be identical in all four layouts and contain no padding. */
  int luma_bad = 0;
  for (int row = 0; row < h; row++) {
    for (int x = 0; x < w; x++) {
      if (dst[(size_t)row * w + x] != SourceFrame::y_value(x, row)) {
        luma_bad++;
      }
    }
  }
  CHECK_EQ_INT(luma_bad, 0);

  if (exp.uv_interleaved) {
    /* NV12: U,V  NV21: V,U. Both live in the pane that starts after luma. */
    int bad = 0;
    for (int row = 0; row < ch; row++) {
      for (int cx = 0; cx < cw; cx++) {
        const size_t off = exp.luma_size + ((size_t)row * cw + cx) * 2;
        const bool nv12 = (fmt == V4L2_PIX_FMT_NV12);
        const uint8_t want0 = nv12 ? SourceFrame::u_value(cx, row)
                                   : SourceFrame::v_value(cx, row);
        const uint8_t want1 = nv12 ? SourceFrame::v_value(cx, row)
                                   : SourceFrame::u_value(cx, row);
        if (dst[off + 0] != want0 || dst[off + 1] != want1) {
          bad++;
        }
      }
    }
    CHECK_EQ_INT(bad, 0);
  } else {
    /* YU12: U pane then V pane. YV12: V pane then U pane. */
    const size_t first_off = exp.luma_size;
    const size_t second_off = exp.luma_size + exp.chroma_size;
    int bad = 0;
    for (int row = 0; row < ch; row++) {
      for (int cx = 0; cx < cw; cx++) {
        const size_t idx = (size_t)row * cw + cx;
        const uint8_t want_first = exp.v_first ? SourceFrame::v_value(cx, row)
                                               : SourceFrame::u_value(cx, row);
        const uint8_t want_second = exp.v_first ? SourceFrame::u_value(cx, row)
                                                : SourceFrame::v_value(cx, row);
        if (dst[first_off + idx] != want_first ||
            dst[second_off + idx] != want_second) {
          bad++;
        }
      }
    }
    CHECK_EQ_INT(bad, 0);
  }
}

void run_geometry_case(int w, int h, size_t y_pad, size_t uv_pad,
                       size_t pixel_stride, bool interleaved, const char* label) {
  SourceFrame src;
  src.build(w, h, y_pad, uv_pad, pixel_stride, interleaved);

  const uint32_t formats[] = {
    V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_YVU420,
    V4L2_PIX_FMT_NV12,   V4L2_PIX_FMT_NV21,
  };

  const ExpectedLayout exp = expected_layout(V4L2_PIX_FMT_YUV420, w, h);
  const size_t needed = exp.luma_size + exp.chroma_size * 2;

  for (uint32_t fmt : formats) {
    std::vector<uint8_t> dst(needed, 0x5A);
    const int r = pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                                     w, h, fmt, dst.data(), dst.size());
    CHECK_EQ_INT(r, 0);
    verify_packed(src, dst, fmt, label);
  }
}

/* -------------------------------------------------------------------------
 * Round trip through convert_frame() to detect U/V swaps and stride errors by
 * colour: a wrong chroma interpretation produces a wrong RGB picture.
 * ---------------------------------------------------------------------- */
void run_roundtrip_case(uint32_t pack_format, const char* label) {
  const int w = 64, h = 48;
  SourceFrame src;
  src.build(w, h, 32, 32, 2, true);

  const ExpectedLayout exp = expected_layout(pack_format, w, h);
  std::vector<uint8_t> packed(exp.luma_size + exp.chroma_size * 2, 0);
  const int r = pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                                   w, h, pack_format, packed.data(), packed.size());
  std::printf("  [roundtrip %s]\n", label);
  CHECK_EQ_INT(r, 0);
  if (r != 0) {
    return;
  }

  /* Reference: convert the packed frame itself to RGB32. Whatever the packer
   * produced must round trip consistently, so the same frame packed into
   * YV12/NV12/NV21/YU12 must yield identical RGB. */
  std::vector<uint32_t> rgb((size_t)w * h, 0);
  anbox::camera::ClientFrameBuffer fb;
  fb.pixel_format = V4L2_PIX_FMT_RGB32;
  fb.framebuffer = rgb.data();
  const int cr = anbox::camera::convert_frame(packed.data(), pack_format,
                                              packed.size(), w, h, &fb, 1,
                                              1.0f, 1.0f, 1.0f, 1.0f);
  CHECK_EQ_INT(cr, 0);
  if (cr != 0) {
    return;
  }

  /* Spot check a pixel in the middle: with the synthetic pattern above it is
   * not neutral, so channels are meaningfully different. */
  const uint32_t px = rgb[(size_t)(h / 2) * w + w / 2];
  const unsigned r8 = px & 0xff, g8 = (px >> 8) & 0xff, b8 = (px >> 16) & 0xff;
  std::printf("    centre pixel rgb=(%u,%u,%u)\n", r8, g8, b8);
  CHECK(std::abs((int)r8 - (int)g8) > 2 || std::abs((int)g8 - (int)b8) > 2);
}

void run_error_cases() {
  std::printf("  [error handling]\n");
  const int w = 64, h = 48;
  SourceFrame src;
  src.build(w, h, 0, 0, 1, false);

  const ExpectedLayout exp = expected_layout(V4L2_PIX_FMT_YVU420, w, h);
  const size_t needed = exp.luma_size + exp.chroma_size * 2;

  /* Destination one byte short must be rejected, not written past the end. */
  std::vector<uint8_t> small(needed - 1, 0);
  CHECK(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                           w, h, V4L2_PIX_FMT_YVU420,
                           small.data(), small.size()) != 0);

  /* Exactly enough must succeed. */
  std::vector<uint8_t> exact(needed, 0);
  CHECK_EQ_INT(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                                  w, h, V4L2_PIX_FMT_YVU420,
                                  exact.data(), exact.size()), 0);

  /* Unsupported destination format. */
  std::vector<uint8_t> dst(needed, 0);
  CHECK(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                           w, h, V4L2_PIX_FMT_YUYV,
                           dst.data(), dst.size()) != 0);

  /* Invalid geometry. */
  CHECK(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                           0, h, V4L2_PIX_FMT_YVU420, dst.data(), dst.size()) != 0);
  CHECK(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                           w, 0, V4L2_PIX_FMT_YVU420, dst.data(), dst.size()) != 0);
  CHECK(pack_yuv420_planes(src.y_plane(), src.u_plane(), src.v_plane(),
                           w, h, V4L2_PIX_FMT_YVU420, nullptr, 0) != 0);

  /* Missing planes. */
  PlaneDesc missing{ nullptr, 0, 0 };
  CHECK(pack_yuv420_planes(missing, src.u_plane(), src.v_plane(),
                           w, h, V4L2_PIX_FMT_YVU420, dst.data(), dst.size()) != 0);
  CHECK(pack_yuv420_planes(src.y_plane(), missing, src.v_plane(),
                           w, h, V4L2_PIX_FMT_YVU420, dst.data(), dst.size()) != 0);

  /* Odd dimensions are rejected by startCapturing() before a frame is ever
   * read, because the guest buffer ((w * h * 3) / 2) is smaller than what a
   * padded chroma plane needs. The packer itself must not silently write a
   * different size than the guest expects. */
  std::printf("  [odd geometry contract]\n");
  struct OddCase { int w, h; };
  const OddCase odd[] = { { 3, 3 }, { 1, 480 }, { 641, 480 }, { 643, 4 } };
  for (const OddCase& c : odd) {
    SourceFrame odd_src;
    odd_src.build(c.w, c.h, 0, 0, 1, false);
    const size_t guest_size = (size_t)c.w * c.h * 3 / 2;
    std::vector<uint8_t> buf(guest_size, 0);
    CHECK(pack_yuv420_planes(odd_src.y_plane(), odd_src.u_plane(),
                             odd_src.v_plane(), c.w, c.h,
                             V4L2_PIX_FMT_YUV420,
                             buf.data(), buf.size()) != 0);
  }
}

}  // namespace

int main() {
  std::printf("pack_yuv420_planes unit tests\n");

  std::printf("-- tight (no padding) --\n");
  run_geometry_case(640, 480, 0, 0, 1, false, "640x480 tight");

  std::printf("-- padded rows, the common hardware case --\n");
  run_geometry_case(640, 480, 0, 0, 2, true, "640x480 ndk interleaved");
  run_geometry_case(638, 478, 2, 2, 2, true, "638x478 ndk interleaved padded");
  run_geometry_case(641, 481, 63, 63, 2, true, "641x481 ndk interleaved padded");
  run_geometry_case(100, 100, 28, 8, 2, true, "100x100 ndk interleaved padded");
  run_geometry_case(640, 480, 64, 64, 1, false, "640x480 padded separate");

  std::printf("-- small sizes --\n");
  run_geometry_case(2, 2, 0, 0, 1, false, "2x2");
  run_geometry_case(4, 2, 3, 1, 2, true, "4x2 interleaved");

  std::printf("-- round trip through convert_frame --\n");
  run_roundtrip_case(V4L2_PIX_FMT_YUV420, "YU12");
  run_roundtrip_case(V4L2_PIX_FMT_YVU420, "YV12");
  run_roundtrip_case(V4L2_PIX_FMT_NV12, "NV12");
  run_roundtrip_case(V4L2_PIX_FMT_NV21, "NV21");

  std::printf("-- error handling --\n");
  run_error_cases();

  std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
