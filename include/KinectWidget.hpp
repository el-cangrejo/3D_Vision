/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#ifndef KINECTWIDGET_H
#define KINECTWIDGET_H

#include "libfreenect/libfreenect.hpp"

#include <QGLWidget>
#include <QMouseEvent>

#include <cmath>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

class Mutex {
public:
  Mutex() { pthread_mutex_init(&m_mutex, NULL); }
  void lock() { pthread_mutex_lock(&m_mutex); }
  void unlock() { pthread_mutex_unlock(&m_mutex); }

  class ScopedLock {
    Mutex &_mutex;

  public:
    ScopedLock(Mutex &mutex) : _mutex(mutex) { _mutex.lock(); }
    ~ScopedLock() { _mutex.unlock(); }
  };

private:
  pthread_mutex_t m_mutex;
};

/* thanks to Yoda---- from IRC */
class MyFreenectDevice : public Freenect::FreenectDevice {
public:
  MyFreenectDevice(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index),
        m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
                                                FREENECT_VIDEO_RGB)
                           .bytes),
        m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
                                                FREENECT_VIDEO_RGB)
                           .bytes),
        m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false) {
    for (unsigned int i = 0; i < 2048; i++) {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      m_gamma[i] = v * 6 * 256;
    }
  }
  //~MyFreenectDevice(){}
  // Do not call directly even in child
  void VideoCallback(void *_rgb, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_rgb_mutex);
    uint8_t *rgb = static_cast<uint8_t *>(_rgb);
    std::copy(rgb, rgb + getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
  };
  // Do not call directly even in child
  void DepthCallback(void *_depth, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_depth_mutex);
    uint16_t *depth = static_cast<uint16_t *>(_depth);
    for (unsigned int i = 0; i < 640 * 480; i++) {
      int pval = m_gamma[depth[i]];
      int lb = pval & 0xff;
      switch (pval >> 8) {
      case 0:
        m_buffer_depth[3 * i + 0] = 255;
        m_buffer_depth[3 * i + 1] = 255 - lb;
        m_buffer_depth[3 * i + 2] = 255 - lb;
        break;
      case 1:
        m_buffer_depth[3 * i + 0] = 255;
        m_buffer_depth[3 * i + 1] = lb;
        m_buffer_depth[3 * i + 2] = 0;
        break;
      case 2:
        m_buffer_depth[3 * i + 0] = 255 - lb;
        m_buffer_depth[3 * i + 1] = 255;
        m_buffer_depth[3 * i + 2] = 0;
        break;
      case 3:
        m_buffer_depth[3 * i + 0] = 0;
        m_buffer_depth[3 * i + 1] = 255;
        m_buffer_depth[3 * i + 2] = lb;
        break;
      case 4:
        m_buffer_depth[3 * i + 0] = 0;
        m_buffer_depth[3 * i + 1] = 255 - lb;
        m_buffer_depth[3 * i + 2] = 255;
        break;
      case 5:
        m_buffer_depth[3 * i + 0] = 0;
        m_buffer_depth[3 * i + 1] = 0;
        m_buffer_depth[3 * i + 2] = 255 - lb;
        break;
      default:
        m_buffer_depth[3 * i + 0] = 0;
        m_buffer_depth[3 * i + 1] = 0;
        m_buffer_depth[3 * i + 2] = 0;
        break;
      }
    }
    m_new_depth_frame = true;
  }
  bool getRGB(std::vector<uint8_t> &buffer) {
    Mutex::ScopedLock lock(m_rgb_mutex);
    if (!m_new_rgb_frame)
      return false;
    buffer.swap(m_buffer_video);
    m_new_rgb_frame = false;
    return true;
  }

  bool getDepth(std::vector<uint8_t> &buffer) {
    Mutex::ScopedLock lock(m_depth_mutex);
    if (!m_new_depth_frame)
      return false;
    buffer.swap(m_buffer_depth);
    m_new_depth_frame = false;
    return true;
  }

private:
  std::vector<uint8_t> m_buffer_depth;
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_gamma;
  Mutex m_rgb_mutex;
  Mutex m_depth_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
};

class KinectWidget : public QGLWidget {
public:
  explicit KinectWidget(QWidget *parent = 0);

  // GL Functions
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);
  void timerEvent(QTimerEvent *);

public:
  Freenect::Freenect freenect;
  MyFreenectDevice *device;
  // freenect_video_format requested_format(FREENECT_VIDEO_RGB);

  GLuint gl_depth_tex;
  GLuint gl_rgb_tex;

  // double freenect_angle(0);
  int got_frames, window;
};

#endif // KINECTWIDGET_H
