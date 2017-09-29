#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "Open3DOR.hpp"

#include <QGLWidget>
#include <QMouseEvent>

#define PI 3.1415927

class GLWidget : public QGLWidget {

public:
  explicit GLWidget(QWidget *parent = 0);

  // GL Functions
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);

  // Event Handlers
  void mousePressEvent(QMouseEvent *qevent);
  void mouseMoveEvent(QMouseEvent *qevent);
  void wheelEvent(QWheelEvent *qevent);
  void keyPressEvent(QKeyEvent *qevent);

  // Rendering Functions
  void draw_mesh(Mesh &);
  void draw_grid(Mesh &);
  void draw_axis();

  // Color Setting Functions
  void setModelColorR(float);
  void setModelColorB(float);
  void setModelColorG(float);

  void setBackgroundColorR(float);
  void setBackgroundColorB(float);
  void setBackgroundColorG(float);

  // Rendering Options Setting Functions
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

  // Database Handling Functions
  void eraseDatabase();

  // Rendering Options
  bool _showVerts = true;
  bool _showTriangles = false;
  bool _showNormals = false;
  bool _showFilteredMesh = false;
  bool _showWire = false;
  bool _showGrid = false;
  bool _showSolid = true;
  bool _showTargetMesh = false;
  bool _showAxis = true;
  bool _normalLighting = false;
  bool _modelLighting = false;
	bool _showMultiMesh = false;
	bool _showDatabase = false; 

  float _BackgroundColorR = 0.13;
  float _BackgroundColorG = 0.7;
  float _BackgroundColorB = 0.77;

  float _ModelColorR = 0.8;
  float _ModelColorG = 0.76;
  float _ModelColorB = 0.78;

  float _NormalsLength = 0.0;
  float _zoomStep = 0.1;
  float _rotFactor = 0.01;

  Vertex up_vector;

  Vertex sphere;
  float sphere_r = 0.05;
  float sphere_color = 0.0;

  Mesh primary_mesh;
  Mesh filtered_mesh;
  Mesh target_mesh;

  std::vector<Mesh> primary_meshes;
  std::vector<Mesh> database_meshes;
  std::vector<Mesh> meshes;
  std::vector<Mesh> db_descriptors;
  std::vector<std::string> db_files;
  std::string database;

private:
  // Camera Variables
  float cdist = 3.5;
  float eyex = 0.0;
  float eyey = 0.0;
  float targetx = 0.0;
  float targety = 0.0;
  float targetz = 0.0;
  float eyez = cdist;
  float azimuthAngle = 0.0;
  float altitudeAngle = PI / 2.;

  // Event Handle Variables
  bool mouseClickDown = false;
  int mx0 = 0;
  int my0 = 0;
};

#endif // GLWIDGET_H
