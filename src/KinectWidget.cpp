#include "KinectWidget.hpp"

KinectWidget::KinectWidget(QWidget *parent)
    : QGLWidget(parent), kinect_initialized(false),
      depthMat(cv::Size(640, 480), CV_16UC1) {}

void KinectWidget::initializeGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glGenTextures(1, &gl_depth_tex);
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glGenTextures(1, &gl_rgb_tex);
  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void KinectWidget::paintGL() {

  static std::vector<uint8_t> depth(640 * 480 * 4, 0);
  static std::vector<uint8_t> rgb(640 * 480 * 4, 0);

  if (kinect_initialized) {
    device->updateState();

    device->getDepth(depth, depthMat);
    device->getRGB(rgb);
  }

  got_frames = 0;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE,
               &depth[0]);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(width_, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(width_, height_ / 2.0, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, height_ / 2.0, 0);
  glEnd();

  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE,
               &rgb[0]);
  //  if (device->getVideoFormat() == FREENECT_VIDEO_RGB ||
  //      device->getVideoFormat() == FREENECT_VIDEO_YUV_RGB)
  //    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE,
  //                 &rgb[0]);
  //  else
  //    glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE,
  //                 GL_UNSIGNED_BYTE, &rgb[0]);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, height_ / 2.0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(width_, height_ / 2.0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(width_, height_, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, height_, 0);
  glEnd();
}

void KinectWidget::resizeGL(int w, int h) {
  width_ = w;
  height_ = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, w, h, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

void KinectWidget::timerEvent(QTimerEvent *event) { updateGL(); }
