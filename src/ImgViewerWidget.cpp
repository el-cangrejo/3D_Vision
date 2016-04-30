#include "ImgViewerWidget.hpp"

ImgViewerWidget::ImgViewerWidget(QWidget *parent) : QGLWidget(parent) {}

void ImgViewerWidget::initializeGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glGenTextures(1, &gl_img_tex);
  glBindTexture(GL_TEXTURE_2D, gl_img_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void ImgViewerWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, gl_img_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, img.cols, img.rows, 0, GL_RGB,
               GL_UNSIGNED_BYTE, img.data);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(width_, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(width_, height_, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, height_, 0);
  glEnd();
}

void ImgViewerWidget::resizeGL(int w, int h) {
  width_ = w;
  height_ = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, w, h, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

void ImgViewerWidget::setImg(cv::Mat depthMat) {
  img = depthMat;
  updateGL();
  // setMinimumHeight(img.rows);
  // setMinimumWidth(img.cols);
}

cv::Mat ImgViewerWidget::getImg() { return img; }
