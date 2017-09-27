#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  bool zoom;
  ~MainWindow();

private slots:
  void on_StatOutFIlter_clicked();

  void on_GridSize_valueChanged(double arg1);

  void on_NormalsLengthSlider_valueChanged(int value);

  void on_ZoomFactSlider_sliderMoved(int position);

  void on_RotFactorSlider_sliderMoved(int position);

  void on_SearchinDBButton_clicked();

  /*
   * Menu Actions
   * */

  void onActionQuit();
  void onActionOpenMesh();
  void onActionOpenImage8U();
  void onActionOpenImage16U();
  void onActionOpenImage(int type);
  void onActionOpenKinect();
  void onActionLoadDatabase();
  void onActionPreprocessDatabase();
  void onActionTakeSnapshot();

  /*
   * Color Setting Functions
   * Sliders
   * */

  void setModelColorR(int);
  void setModelColorG(int);
  void setModelColorB(int);

  void setBackgroundColorR(int);
  void setBackgroundColorB(int);
  void setBackgroundColorG(int);

  /*
   * Rendering Options Setting Functions
   * Check Boxes
   * */

  void setShowVertices(bool);
  void setShowTriangles(bool);
  void setShowWire(bool);
  void setShowNormals(bool);
  void setShowGrid(bool);
  void setShowSolid(bool);
  void setShowAxis(bool);
  void setShowFilteredMesh(bool);
  void setShowTargetMesh(bool);
  void setModelLighting(bool);
  void setNormalsLighting(bool);
	void setMultiMesh(bool);
	void setShowDatabase(bool);

  void onSegmentImg();
  void onGenerateMesh();
  void onDraw();
  void onClearAll();
  void onZoomImgWidget();
  void gridFilter();
	void onComputeDescriptors();
	void onRetrieve();

  /*
   * Segmentation Parameters
   * */

  void setMedianKernel(int );
  void setNormalsRadius(int );
  void setEdgeRadius(int );
  void setKernelRadius(int );

private:
  Ui::MainWindow *ui;

  void inititializeUI();
  void connectUI();

  float median_kernel;
  float normals_radius;
  float edge_radius;
	float kernel_radius;
};

#endif // MAINWINDOW_H
