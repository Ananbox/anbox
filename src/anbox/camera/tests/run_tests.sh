#!/bin/sh
# Builds and runs the pack_yuv420_planes unit test on a plain host.
#
# The test needs three things that a desktop machine usually does not have:
#   - <linux/videodev2.h>  (Linux kernel header)
#   - boost                (pulled in by anbox/logger.h)
#   - anbox's logger implementation
# Instead of faking the whole build, this script provides a minimal
# videodev2.h with only the fourcc constants the packer uses (their values are
# fixed by the V4L2 ABI) and a logger stub that forwards to printf.
#
# Usage: ./run_tests.sh [compiler]

set -e

CC=${1:-c++}
HERE=$(cd "$(dirname "$0")" && pwd)
ANBOX_SRC=$(cd "$HERE/../../../.." && pwd)
BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT

mkdir -p "$BUILD/shim/linux" "$BUILD/shim/anbox"

cat > "$BUILD/shim/linux/videodev2.h" <<'EOF'
/* Minimal stand-in for <linux/videodev2.h>. Only the fourcc constants used by
 * the packer are needed; the values are part of the V4L2 ABI. */
#ifndef ANBOX_TEST_SHIM_VIDEODEV2_H
#define ANBOX_TEST_SHIM_VIDEODEV2_H
#include <stdint.h>
#define v4l2_fourcc(a, b, c, d) \
  ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define V4L2_PIX_FMT_RGB32   v4l2_fourcc('B', 'G', 'R', '4')
#define V4L2_PIX_FMT_BGR32   v4l2_fourcc('B', 'G', 'R', '4')
#define V4L2_PIX_FMT_RGB24   v4l2_fourcc('B', 'G', 'R', '3')
#define V4L2_PIX_FMT_BGR24   v4l2_fourcc('B', 'G', 'R', '3')
#define V4L2_PIX_FMT_RGB565  v4l2_fourcc('R', 'G', 'B', 'P')
#define V4L2_PIX_FMT_YUYV    v4l2_fourcc('Y', 'U', 'Y', 'V')
#define V4L2_PIX_FMT_UYVY    v4l2_fourcc('U', 'Y', 'V', 'Y')
#define V4L2_PIX_FMT_YVYU    v4l2_fourcc('Y', 'V', 'Y', 'U')
#define V4L2_PIX_FMT_VYUY    v4l2_fourcc('V', 'Y', 'U', 'Y')
#define V4L2_PIX_FMT_YYUV    v4l2_fourcc('Y', 'Y', 'U', 'V')
#define V4L2_PIX_FMT_YYVU    v4l2_fourcc('Y', 'Y', 'V', 'U')
#define V4L2_PIX_FMT_YUY2    v4l2_fourcc('Y', 'U', 'Y', '2')
#define V4L2_PIX_FMT_YUNV    v4l2_fourcc('Y', 'U', 'N', 'V')
#define V4L2_PIX_FMT_V422    v4l2_fourcc('V', '4', '2', '2')
#define V4L2_PIX_FMT_YUV420  v4l2_fourcc('Y', 'U', '1', '2')
#define V4L2_PIX_FMT_YVU420  v4l2_fourcc('Y', 'V', '1', '2')
#define V4L2_PIX_FMT_NV12    v4l2_fourcc('N', 'V', '1', '2')
#define V4L2_PIX_FMT_NV21    v4l2_fourcc('N', 'V', '2', '1')
#define V4L2_PIX_FMT_SBGGR8  v4l2_fourcc('B', 'A', '8', '1')
#define V4L2_PIX_FMT_SGBRG8  v4l2_fourcc('G', 'B', 'R', 'G')
#define V4L2_PIX_FMT_SGRBG8  v4l2_fourcc('G', 'R', 'B', 'G')
#define V4L2_PIX_FMT_SRGGB8  v4l2_fourcc('R', 'G', 'G', 'B')
#define V4L2_PIX_FMT_SBGGR10 v4l2_fourcc('B', 'G', '1', '0')
#define V4L2_PIX_FMT_SGBRG10 v4l2_fourcc('G', 'B', '1', '0')
#define V4L2_PIX_FMT_SGRBG10 v4l2_fourcc('B', 'A', '1', '0')
#define V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R', 'G', '1', '0')
#define V4L2_PIX_FMT_SBGGR12 v4l2_fourcc('B', 'G', '1', '2')
#define V4L2_PIX_FMT_SGBRG12 v4l2_fourcc('G', 'B', '1', '2')
#define V4L2_PIX_FMT_SGRBG12 v4l2_fourcc('B', 'A', '1', '2')
#define V4L2_PIX_FMT_SRGGB12 v4l2_fourcc('R', 'G', '1', '2')
#endif
EOF

cat > "$BUILD/shim/anbox/logger.h" <<'EOF'
/* Logger stub: the converters only use ERROR/WARNING/DEBUG for diagnostics. */
#ifndef ANBOX_TEST_SHIM_LOGGER_H
#define ANBOX_TEST_SHIM_LOGGER_H
#include <cstdarg>
#include <cstdio>
inline void anbox_test_log(const char* level, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  std::printf("  [%s] ", level);
  std::vprintf(fmt, ap);
  std::printf("\n");
  va_end(ap);
}
#define ERROR(...)   anbox_test_log("error", __VA_ARGS__)
#define WARNING(...) anbox_test_log("warn", __VA_ARGS__)
#define DEBUG(...)   anbox_test_log("debug", __VA_ARGS__)
#endif
EOF

echo "building..."
"$CC" -std=c++14 -O1 -g -fsanitize=address,undefined \
    -Wall -Wextra -Wno-unused-function -Wno-unused-const-variable \
    -I"$BUILD/shim" \
    -I"$ANBOX_SRC/src" \
    "$HERE/test_pack_yuv420_planes.cpp" \
    "$ANBOX_SRC/src/anbox/camera/camera_format_converters.cpp" \
    -o "$BUILD/pack_test"

echo "running..."
"$BUILD/pack_test"
