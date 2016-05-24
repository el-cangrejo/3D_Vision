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
  void mousePressEvent(QMouseEvent *qevent);
  void mouseMoveEvent(QMouseEvent *qevent);
  // Set Image to Display
  void setImg(cv::Mat depthMat);
  void clearImg();

  // Get Displayed Image
  cv::Mat getImg();

  cv::Vec3b mask;
  bool mask_bool;
  bool draw;
  bool mouseClickDown;
private:
  cv::Mat img;
  GLuint gl_img_tex;
  int width_, height_;
};

#endif // IMGVIEWERWIDGET_H
