#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "Open3DOR.hpp"

#include <Eigen/Dense>

#include <QGLWidget>
#include <QMouseEvent>

#define PI 3.1415927

class GLWidget : public QGLWidget
{

public:
    explicit GLWidget(QWidget *parent = 0);

    // GL functions
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    // Event handlers
    void mousePressEvent(QMouseEvent *qevent);
    void mouseMoveEvent(QMouseEvent *qevent);
    void wheelEvent(QWheelEvent *);

    // Rendering functions
    void draw_mesh(Mesh&);
    void draw_grid(Mesh&);
    void draw_axis();

    // Color setting functions
    void setModelColorR (float);
    void setModelColorB (float);
    void setModelColorG (float);

    void setBackgroundColorR (float);
    void setBackgroundColorB (float);
    void setBackgroundColorG (float);

    // Rendering options setting functions
    void setShowVertices (bool);
    void setShowTriangles (bool);
    void setShowWire (bool);
    void setShowNormals (bool);
    void setShowGrid (bool);
    void setShowSolid (bool);
    void setShowAxis (bool);
    void setShowTargetMesh (bool);

    void eraseDatabase ();

    // Rendering options
    bool _showGrid = false;
    bool _showVerts = false;
    bool _showTriangles = true;
    bool _showNormals = false;
    bool _showFilteredMesh = false;
    bool _showWire = false;
    bool _showSolid = true;
    bool _showTargetMesh = false;
    bool _showAxis = true;
    bool _normalLighting = false;
    bool _enableLighting = false;

    float _BackgroundColorR = 0.8;
    float _BackgroundColorG = 0.8;
    float _BackgroundColorB = 0.8;

    float _ModelColorR = 0.1;
    float _ModelColorG = 0.0;
    float _ModelColorB = 1.0;

    float _NormalsLength = 0.0;
    float _zoomStep = 0.01;
    float _rotFactor = 0.01;

    float _width;
    float _height;

    Vertex up_vector;

    Vertex sphere;
    float sphere_r = 0.05;
    float sphere_color = 0.0;

    Mesh primary_mesh;
    Mesh filtered_mesh;
    Mesh target_mesh;

    std::vector<Mesh> db_descriptors;
    std::vector<std::string> db_files;
    std::string database;
private:
    // Camera variables
    float cdist = 3.5;
    float eyex = 0.0;
    float eyey = 0.0;
    float eyez = cdist;
    float azimuthAngle = 0.0;
    float altitudeAngle = PI/2.;

    // Event handle variables
    bool mouseClickDown = false;
    int mx0 = 0;
    int my0 = 0;
};

#endif // GLWIDGET_H
