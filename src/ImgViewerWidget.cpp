#include "ImgViewerWidget.hpp"
#include "Segmentation.hpp"

#include <GL/glu.h>
#include <GL/glut.h>


ImgViewerWidget::ImgViewerWidget(QWidget *parent) : QGLWidget(parent),
            mask_bool(false), draw(false) {}

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
                            img.step, QImage::Format_RGB888);
      type = GL_RGB;
  }
  else if( img.channels() == 1) {
      image = QImage((const unsigned char*)(img.data),
                            img.cols, img.rows,
                            img.step, QImage::Format_Indexed8);
  }
  
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
  updateGL();
}

cv::Mat ImgViewerWidget::getImg() { return img; }

void ImgViewerWidget::clearImg() {
  img.release();
}

void ImgViewerWidget::mousePressEvent(QMouseEvent *qevent) {
  if (qevent->button() == Qt::LeftButton) {
    mouseClickDown = true;
    int c = ceil((qevent->x() * img.cols ) / width_);
    int r = ceil((qevent->y() * img.rows ) / height_);
    std::cout << "Color : " << (int)img.at<cv::Vec3b>(r, c)[0] << " "
              << (int)img.at<cv::Vec3b>(r, c)[1] << " "
              << (int)img.at<cv::Vec3b>(r, c)[2] << "\n";
    std::cout << "Pixel : " << r << " " << c << "\n";
    mask = img.at<cv::Vec3b>(r, c);
    mask_bool = true;
    if (draw) {
      cv::circle(img, cv::Point(c, r), 15, cv::Scalar(255, 255, 255), -1);
      updateGL();
    } else {
        mouseClickDown = false;
    }
  }
}

void ImgViewerWidget::mouseMoveEvent(QMouseEvent *qevent) {
  if (mouseClickDown) {
    int c = ceil((qevent->x() * img.cols ) / width_);
    int r = ceil((qevent->y() * img.rows ) / height_);
    std::cout << "Color : " << (int)img.at<cv::Vec3b>(r, c)[0] << " "
              << (int)img.at<cv::Vec3b>(r, c)[1] << " "
              << (int)img.at<cv::Vec3b>(r, c)[2] << "\n";
    std::cout << "Pixel : " << r << " " << c << "\n";
    mask = img.at<cv::Vec3b>(r, c);
    mask_bool = true;
    if (draw) {
      cv::circle(img, cv::Point(c, r), 15, cv::Scalar(255, 255, 255), -1);
      updateGL();
    }
  }
}
