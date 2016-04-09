#include "KinectWidget.hpp"

KinectWidget::KinectWidget(QWidget *parent) : QGLWidget(parent) {
  startTimer(10);
}

void KinectWidget::initializeGL() {
  device = &freenect.createDevice<MyFreenectDevice>(0);
  device->startVideo();
  device->startDepth();
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
  // ReSizeGLScene(Width, Height);
}

void KinectWidget::paintGL() {
  static std::vector<uint8_t> depth(640 * 480 * 4);
  static std::vector<uint8_t> rgb(640 * 480 * 4);

  // using getTiltDegs() in a closed loop is unstable
  /*if(device->getState().m_code == TILT_STATUS_STOPPED){
    freenect_angle = device->getState().getTiltDegs();
  }*/
  device->updateState();
  // printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f",
  // freenect_angle, device->getState().getTiltDegs());
  fflush(stdout);

  device->getDepth(depth);
  device->getRGB(rgb);

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
  glVertex3f(640, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(640, 480, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, 480, 0);
  glEnd();

  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  if (device->getVideoFormat() == FREENECT_VIDEO_RGB ||
      device->getVideoFormat() == FREENECT_VIDEO_YUV_RGB)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 &rgb[0]);
  else
    glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE,
                 GL_UNSIGNED_BYTE, &rgb[0]);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(640, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(1280, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(1280, 480, 0);
  glTexCoord2f(0, 1);
  glVertex3f(640, 480, 0);
  glEnd();

  // glutSwapBuffers();
}

void KinectWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, w, h, 0, -1.0f, 1.0f);
  // gluPerspective(50.0, (float)width / height, 900.0, 11000.0);
  glMatrixMode(GL_MODELVIEW);
}

void KinectWidget::timerEvent(QTimerEvent *event) { updateGL(); }
