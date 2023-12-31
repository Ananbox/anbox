/*
* Copyright (C) 2015 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "anbox/graphics/emugl/TextureResize.h"
#include "anbox/graphics/emugl/DispatchTables.h"
#include "anbox/logger.h"

#include <stdio.h>
#include <sstream>
#include <string>
#include <utility>

#define MAX_FACTOR_POWER 4

static const char kCommonShaderSource[] =
    "precision mediump float;\n"
    "varying vec2 vUV00, vUV01;\n"
    "#if FACTOR > 2\n"
    "varying vec2 vUV02, vUV03;\n"
    "#if FACTOR > 4\n"
    "varying vec2 vUV04, vUV05, vUV06, vUV07;\n"
    "#if FACTOR > 8\n"
    "varying vec2 vUV08, vUV09, vUV10, vUV11, vUV12, vUV13, vUV14, vUV15;\n"
    "#endif\n"
    "#endif\n"
    "#endif\n";

static const char kVertexShaderSource[] =
    "attribute vec2 aPosition;\n"

    "void main() {\n"
    "  gl_Position = vec4(aPosition, 0, 1);\n"
    "  vec2 uv = ((aPosition + 1.0) / 2.0) + 0.5 / kDimension;\n"
    "  vUV00 = uv;\n"
    "  #ifdef HORIZONTAL\n"
    "  vUV01 = uv + vec2( 1.0 / kDimension.x, 0);\n"
    "  #if FACTOR > 2\n"
    "  vUV02 = uv + vec2( 2.0 / kDimension.x, 0);\n"
    "  vUV03 = uv + vec2( 3.0 / kDimension.x, 0);\n"
    "  #if FACTOR > 4\n"
    "  vUV04 = uv + vec2( 4.0 / kDimension.x, 0);\n"
    "  vUV05 = uv + vec2( 5.0 / kDimension.x, 0);\n"
    "  vUV06 = uv + vec2( 6.0 / kDimension.x, 0);\n"
    "  vUV07 = uv + vec2( 7.0 / kDimension.x, 0);\n"
    "  #if FACTOR > 8\n"
    "  vUV08 = uv + vec2( 8.0 / kDimension.x, 0);\n"
    "  vUV09 = uv + vec2( 9.0 / kDimension.x, 0);\n"
    "  vUV10 = uv + vec2(10.0 / kDimension.x, 0);\n"
    "  vUV11 = uv + vec2(11.0 / kDimension.x, 0);\n"
    "  vUV12 = uv + vec2(12.0 / kDimension.x, 0);\n"
    "  vUV13 = uv + vec2(13.0 / kDimension.x, 0);\n"
    "  vUV14 = uv + vec2(14.0 / kDimension.x, 0);\n"
    "  vUV15 = uv + vec2(15.0 / kDimension.x, 0);\n"
    "  #endif\n"  // FACTOR > 8
    "  #endif\n"  // FACTOR > 4
    "  #endif\n"  // FACTOR > 2

    "  #else\n"
    "  vUV01 = uv + vec2(0,  1.0 / kDimension.y);\n"
    "  #if FACTOR > 2\n"
    "  vUV02 = uv + vec2(0,  2.0 / kDimension.y);\n"
    "  vUV03 = uv + vec2(0,  3.0 / kDimension.y);\n"
    "  #if FACTOR > 4\n"
    "  vUV04 = uv + vec2(0,  4.0 / kDimension.y);\n"
    "  vUV05 = uv + vec2(0,  5.0 / kDimension.y);\n"
    "  vUV06 = uv + vec2(0,  6.0 / kDimension.y);\n"
    "  vUV07 = uv + vec2(0,  7.0 / kDimension.y);\n"
    "  #if FACTOR > 8\n"
    "  vUV08 = uv + vec2(0,  8.0 / kDimension.y);\n"
    "  vUV09 = uv + vec2(0,  9.0 / kDimension.y);\n"
    "  vUV10 = uv + vec2(0, 10.0 / kDimension.y);\n"
    "  vUV11 = uv + vec2(0, 11.0 / kDimension.y);\n"
    "  vUV12 = uv + vec2(0, 12.0 / kDimension.y);\n"
    "  vUV13 = uv + vec2(0, 13.0 / kDimension.y);\n"
    "  vUV14 = uv + vec2(0, 14.0 / kDimension.y);\n"
    "  vUV15 = uv + vec2(0, 15.0 / kDimension.y);\n"
    "  #endif\n"  // FACTOR > 8
    "  #endif\n"  // FACTOR > 4
    "  #endif\n"  // FACTOR > 2
    "  #endif\n"  // HORIZONTAL/VERTICAL
    "}\n";

const char kFragmentShaderSource[] =
    "uniform sampler2D uTexture;\n"

    "vec4 read(vec2 uv) {\n"
    "  vec4 r = texture2D(uTexture, uv);\n"
    "  #ifdef HORIZONTAL\n"
    "  r.rgb = pow(r.rgb, vec3(2.2));\n"
    "  #endif\n"
    "  return r;\n"
    "}\n"

    "void main() {\n"
    "  vec4 sum = read(vUV00) + read(vUV01);\n"
    "  #if FACTOR > 2\n"
    "  sum += read(vUV02) + read(vUV03);\n"
    "  #if FACTOR > 4\n"
    "  sum += read(vUV04) + read(vUV05) + read(vUV06) + read(vUV07);\n"
    "  #if FACTOR > 8\n"
    "  sum += read(vUV08) + read(vUV09) + read(vUV10) + read(vUV11) +"
    "      read(vUV12) + read(vUV13) + read(vUV14) + read(vUV15);\n"
    "  #endif\n"
    "  #endif\n"
    "  #endif\n"
    "  sum /= float(FACTOR);\n"
    "  #ifdef VERTICAL\n"
    "  sum.rgb = pow(sum.rgb, vec3(1.0 / 2.2));\n"
    "  #endif\n"
    "  gl_FragColor = sum;\n"
    "}\n";

static const float kVertexData[] = {-1, -1, 3, -1, -1, 3};

static void detachShaders(GLuint program) {
  GLuint shaders[2] = {};
  GLsizei count = 0;
  glGetAttachedShaders(program, 2, &count, shaders);
  if (glGetError() == GL_NO_ERROR) {
    for (GLsizei i = 0; i < count; i++) {
      glDetachShader(program, shaders[i]);
      glDeleteShader(shaders[i]);
    }
  }
}

static GLuint createShader(GLenum type,
                           const std::initializer_list<const char*>& source) {
  GLint success, infoLength;

  GLuint shader = glCreateShader(type);
  if (shader) {
    glShaderSource(shader, source.size(), source.begin(), nullptr);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_FALSE) {
      glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLength);
      std::string infoLog(infoLength + 1, '\0');
      glGetShaderInfoLog(shader, infoLength, nullptr, &infoLog[0]);
      ERROR("%s shader compile failed: %s", (type == GL_VERTEX_SHADER) ? "Vertex" : "Fragment", infoLog.c_str());
      glDeleteShader(shader);
      shader = 0;
    }
  }
  return shader;
}

static void attachShaders(TextureResize::Framebuffer* fb,
                          const char* factorDefine, const char* dimensionDefine,
                          GLuint width, GLuint height) {
  std::ostringstream dimensionConst;
  dimensionConst << "const vec2 kDimension = vec2(" << width << ", " << height
                 << ");\n";

  GLuint vShader = createShader(
      GL_VERTEX_SHADER, {factorDefine, dimensionDefine, kCommonShaderSource,
                         dimensionConst.str().c_str(), kVertexShaderSource});
  GLuint fShader = createShader(
      GL_FRAGMENT_SHADER, {factorDefine, dimensionDefine, kCommonShaderSource,
                           kFragmentShaderSource});

  if (!vShader || !fShader) {
    return;
  }

  glAttachShader(fb->program, vShader);
  glAttachShader(fb->program, fShader);
  glLinkProgram(fb->program);

  glUseProgram(fb->program);
  fb->aPosition = glGetAttribLocation(fb->program, "aPosition");
  fb->uTexture = glGetUniformLocation(fb->program, "uTexture");
}

TextureResize::TextureResize(GLuint width, GLuint height)
    : mWidth(width),
      mHeight(height),
      mFactor(1) {
  glGenTextures(1, &mFBWidth.texture);
  glBindTexture(GL_TEXTURE_2D, mFBWidth.texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glGenTextures(1, &mFBHeight.texture);
  glBindTexture(GL_TEXTURE_2D, mFBHeight.texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glGenFramebuffers(1, &mFBWidth.framebuffer);
  glGenFramebuffers(1, &mFBHeight.framebuffer);

  mFBWidth.program = glCreateProgram();
  mFBHeight.program = glCreateProgram();

  glGenBuffers(1, &mVertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(kVertexData), kVertexData,
                       GL_STATIC_DRAW);
}

TextureResize::~TextureResize() {
  GLuint fb[2] = {mFBWidth.framebuffer, mFBHeight.framebuffer};
  glDeleteFramebuffers(2, fb);

  GLuint tex[2] = {mFBWidth.texture, mFBHeight.texture};
  glDeleteTextures(2, tex);

  glDeleteProgram(mFBWidth.program);
  glDeleteProgram(mFBHeight.program);

  glDeleteBuffers(1, &mVertexBuffer);
}

GLuint TextureResize::update(GLuint texture) {
  // Store the viewport. The viewport is clobbered due to the framebuffers.
  GLint vport[4] = {
      0,
  };
  glGetIntegerv(GL_VIEWPORT, vport);

  // Correctly deal with rotated screens.
  GLint tWidth = vport[2], tHeight = vport[3];
  if ((mWidth < mHeight) != (tWidth < tHeight)) {
    std::swap(tWidth, tHeight);
  }

  // Compute the scaling factor needed to get an image just larger than the
  // target viewport.
  unsigned int factor = 1;
  for (int i = 0, w = mWidth / 2, h = mHeight / 2;
       i < MAX_FACTOR_POWER && w >= tWidth && h >= tHeight;
       i++, w /= 2, h /= 2, factor *= 2) {
  }

  // No resizing needed.
  if (factor == 1) {
    return texture;
  }

  glGetError();  // Clear any GL errors.
  setupFramebuffers(factor);
  resize(texture);
  glViewport(vport[0], vport[1], vport[2],
                     vport[3]);  // Restore the viewport.

  // If there was an error while resizing, just use the unscaled texture.
  GLenum error = glGetError();
  if (error != GL_NO_ERROR) {
    ERROR("GL error while resizing: 0x%x (ignored)", error);
    return texture;
  }

  return mFBHeight.texture;
}

void TextureResize::setupFramebuffers(unsigned int factor) {
  if (factor == mFactor) {
    // The factor hasn't changed, no need to update the framebuffers.
    return;
  }

  // Update the framebuffer sizes to match the new factor.
  glBindTexture(GL_TEXTURE_2D, mFBWidth.texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mWidth / factor, mHeight, 0,
                       GL_RGBA, GL_FLOAT, nullptr);
  glBindFramebuffer(GL_FRAMEBUFFER, mFBWidth.framebuffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                 GL_TEXTURE_2D, mFBWidth.texture, 0);

  glBindTexture(GL_TEXTURE_2D, mFBHeight.texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mWidth / factor,
                       mHeight / factor, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glBindFramebuffer(GL_FRAMEBUFFER, mFBHeight.framebuffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                 GL_TEXTURE_2D, mFBHeight.texture, 0);

  // Update the shaders to the new factor. First detach the old shaders...
  detachShaders(mFBWidth.program);
  detachShaders(mFBHeight.program);

  // ... then attach the new ones.
  std::ostringstream factorDefine;
  factorDefine << "#define FACTOR " << factor << "\n";
  attachShaders(&mFBWidth, factorDefine.str().c_str(), "#define HORIZONTAL\n",
                mWidth, mHeight);
  attachShaders(&mFBHeight, factorDefine.str().c_str(), "#define VERTICAL\n",
                mWidth, mHeight);

  mFactor = factor;
}

void TextureResize::resize(GLuint texture) {
  glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
  glActiveTexture(GL_TEXTURE0);

  // First scale the horizontal dimension by rendering the input texture to a
  // scaled framebuffer.
  glBindFramebuffer(GL_FRAMEBUFFER, mFBWidth.framebuffer);
  glViewport(0, 0, mWidth / mFactor, mHeight);
  glUseProgram(mFBWidth.program);
  glEnableVertexAttribArray(mFBWidth.aPosition);
  glVertexAttribPointer(mFBWidth.aPosition, 2, GL_FLOAT, GL_FALSE, 0,
                                0);
  glBindTexture(GL_TEXTURE_2D, texture);

  // Store the current texture filters and set to nearest for scaling.
  GLint mag_filter, min_filter;
  glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                              &mag_filter);
  glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                              &min_filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glUniform1i(mFBWidth.uTexture, 0);
  glDrawArrays(GL_TRIANGLES, 0,
                       sizeof(kVertexData) / (2 * sizeof(float)));

  // Restore the previous texture filters.
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);

  // Secondly, scale the vertical dimension using the second framebuffer.
  glBindFramebuffer(GL_FRAMEBUFFER, mFBHeight.framebuffer);
  glViewport(0, 0, mWidth / mFactor, mHeight / mFactor);
  glUseProgram(mFBHeight.program);
  glEnableVertexAttribArray(mFBHeight.aPosition);
  glVertexAttribPointer(mFBHeight.aPosition, 2, GL_FLOAT, GL_FALSE, 0,
                                0);
  glBindTexture(GL_TEXTURE_2D, mFBWidth.texture);
  glUniform1i(mFBHeight.uTexture, 0);
  glDrawArrays(GL_TRIANGLES, 0,
                       sizeof(kVertexData) / (2 * sizeof(float)));

  // Clear the bindings.
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glBindTexture(GL_TEXTURE_2D, 0);
}
