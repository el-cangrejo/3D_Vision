#ifndef IMGVIEWERWIDGET_H
#define IMGVIEWERWIDGET_H

#include <opencv2/opencv.hpp>

#include <QGLWidget>
#include <QMouseEvent>

class ImgViewerWidget : public QGLWidget {
public:
  explicit ImgViewerWidget(QWidget *parent = 0);

  // GL Functions
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);
  void setImg(cv::Mat depthMat);
  cv::Mat getImg();

private:
  cv::Mat img;
  GLuint gl_img_tex;
  int width_, height_;
};

#endif // IMGVIEWERWIDGET_H
