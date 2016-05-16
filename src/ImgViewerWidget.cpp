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

  QImage image;

  GLuint type = GL_LUMINANCE;

  if( img.channels() == 3) {
      image = QImage((const unsigned char*)(img.data),
                            img.cols, img.rows,
                            img.step, QImage::Format_RGB888)/*.rgbSwapped()*/;
      type = GL_RGB;
  }
  else if( img.channels() == 1) {
      image = QImage((const unsigned char*)(img.data),
                            img.cols, img.rows,
                            img.step, QImage::Format_Indexed8);
  }

  std::cout << "Image step : " << img.step << "\n";
  
  glBindTexture(GL_TEXTURE_2D, gl_img_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, type,
               GL_UNSIGNED_BYTE, image.bits());



  glBegin(GL_QUADS);
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
  depthMat.copyTo(img);
  std::cout << "Channels set img : " << img.channels() << "\n";
  updateGL();
  // setMinimumHeight(img.rows);
  // setMinimumWidth(img.cols);
}

cv::Mat ImgViewerWidget::getImg() { return img; }
