#include "glwidget.h"
#include <GL/glut.h>
#include <GL/glu.h>

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
}

void GLWidget::initializeGL() {
    glClearColor(_BackgroundColorR, _BackgroundColorG, _BackgroundColorB, 0.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    up_vector = Vertex(0, 1, 0);
    sphere = std::move(Vertex(-0.5, -0.5, -0.5));
}

void GLWidget::paintGL() {
    glClearColor(_BackgroundColorR, _BackgroundColorG, _BackgroundColorB, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eyex, eyey, eyez,  // camera position
              0,  0,  0,         // target position
              up_vector.x,
              up_vector.y,
              up_vector.z);

    float scale_size(1 / 1.5);

    {
    glPushMatrix();
      if ((_showFilteredMesh && !filtered_mesh.empty()) ||
          (_showTargetMesh && !target_mesh.empty())) {
          glTranslatef(0.6, 0, 0);
          glScalef(scale_size, scale_size, scale_size);
      }
      draw_mesh(primary_mesh);
      if (_showGrid)
          draw_grid(primary_mesh);
    glPopMatrix();
    }
    if (_showFilteredMesh && !filtered_mesh.empty()) {
        glPushMatrix();
          glTranslatef(-0.6, 0, 0);
          glScalef(scale_size, scale_size, scale_size);
          draw_mesh(filtered_mesh);
        glPopMatrix();
    }
    if (_showTargetMesh && !target_mesh.empty()) {
        glPushMatrix();
          glTranslatef(-0.6, 0.0, 0.0);
          glScalef(scale_size, scale_size, scale_size);
          draw_mesh(target_mesh);
        glPopMatrix();
    }
    if (_showAxis) {
        glPushMatrix();
        //glTranslatef(-0.5, -0.5, -0.5);
        glTranslatef(sphere.x, sphere.y, sphere.z);
        glScalef(scale_size, scale_size, scale_size);
        draw_axis();
        glPopMatrix();
    }

    /*
    glBegin(GL_QUADS);
    glShadeModel(GL_FLAT);
    glDisable(GL_LIGHTING);
    glColor3f(1.0, 1.0, 1.0);
    for ( int x = -5; x < 5; x++) {
      for ( int z = -5; z < 5; z++)
      {
        glNormal3f(0, 1, 0);
        glVertex3f(x, -1.0, z);
        glNormal3f(0, 1, 0);
        glVertex3f(x, -1.0, z + 1.0);
        glNormal3f(0, 1, 0);
        glVertex3f(x + 1.0, -1.0, z + 1.0);
        glNormal3f(0, 1, 0);
        glVertex3f(x + 1.0, -1.0, z);
      }
    }
    for ( int x = -5; x < 5; x++) {
      for ( int y = -1; y < 5; y++)
      {
        glVertex3f(x, y, -5.0);
        glVertex3f(x, y + 1.0, -5.0);
        glVertex3f(x + 1.0, y + 1.0, -5.0);
        glVertex3f(x + 1.0, y, -5.0);
      }
    }
    glEnable(GL_LIGHTING);
    glEnd();
    //*/
}

void GLWidget::resizeGL(int w, int h) {
    _width = w;
    _height = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w/h, 0.01, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::mousePressEvent(QMouseEvent *qevent) {
    if (qevent->button() == Qt::LeftButton) {
        mouseClickDown = true;
        mx0 = qevent->x();
        my0 = qevent->y();
    } else {
        mouseClickDown = false;
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *qevent) {
    
    if (mouseClickDown) {
      // Calculate angles
      azimuthAngle  -= (qevent->x()-mx0)*_rotFactor;
      altitudeAngle -= (qevent->y()-my0)*_rotFactor;
      // Set new camrea position
      eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
      eyey = cdist*cos(altitudeAngle);
      eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);

      if ((altitudeAngle > 2 * PI) || (altitudeAngle < -2 * PI)) altitudeAngle = 0.0;

      if (altitudeAngle > PI) up_vector.y = -1;
      if (altitudeAngle < PI) up_vector.y = 1;
      if (altitudeAngle < 0) up_vector.y = -1;
      if (altitudeAngle < -PI) up_vector.y = 1;

      // Keep mouse x,y for next call
      mx0 = qevent->x();
      my0 = qevent->y();
      updateGL();
    }
}

void GLWidget::wheelEvent(QWheelEvent *qevent) {
    if (qevent->delta() > 0) {
        cdist -= _zoomStep;
        eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
        eyey = cdist*cos(altitudeAngle);
        eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    } else if (qevent->delta() < 0) {
        cdist += _zoomStep;
        eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
        eyey = cdist*cos(altitudeAngle);
        eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    }
    updateGL();
}

void GLWidget::draw_mesh(Mesh& mesh) {
    glPushMatrix();
    if (_showTriangles && !mesh.triangles.empty()) {
        if (!_showSolid) glDisable(GL_DEPTH_TEST);
        else glEnable(GL_DEPTH_TEST);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glShadeModel(GL_SMOOTH);
        glBegin(GL_TRIANGLES);
        glColor3f(_ModelColorR, _ModelColorG, _ModelColorB);
        for (const auto& t : mesh.triangles) {
            //glColor3f(0.9, 0.1, 0.1);
            glNormal3f(mesh.normals[t.v1].x,
                       mesh.normals[t.v1].y,
                       mesh.normals[t.v1].z);
            glVertex3f(mesh.vertices[t.v1].x,
                       mesh.vertices[t.v1].y,
                       mesh.vertices[t.v1].z);
            //glColor3f(0.1, 0.9, 0.1);
            glNormal3f(mesh.normals[t.v2].x,
                       mesh.normals[t.v2].y,
                       mesh.normals[t.v2].z);
            glVertex3f(mesh.vertices[t.v2].x,
                       mesh.vertices[t.v2].y,
                       mesh.vertices[t.v2].z);
            //glColor3f(0.1, 0.1, 0.9);
            glNormal3f(mesh.normals[t.v3].x,
                       mesh.normals[t.v3].y,
                       mesh.normals[t.v3].z);
            glVertex3f(mesh.vertices[t.v3].x,
                       mesh.vertices[t.v3].y,
                       mesh.vertices[t.v3].z);
        }
        glEnd();

    }
    if (_showWire && !mesh.triangles.empty()) {
        glBegin(GL_LINES);
        glColor3f(0.8, 0.0, 0.0);
        for (const auto& t :mesh.triangles) {
            glNormal3f(mesh.normals[t.v1].x,
                       mesh.normals[t.v1].y,
                       mesh.normals[t.v1].z);
            glVertex3f(mesh.vertices[t.v1].x,
                       mesh.vertices[t.v1].y,
                       mesh.vertices[t.v1].z);
            glNormal3f(mesh.normals[t.v2].x,
                       mesh.normals[t.v2].y,
                       mesh.normals[t.v2].z);
            glVertex3f(mesh.vertices[t.v2].x,
                       mesh.vertices[t.v2].y,
                       mesh.vertices[t.v2].z);

            glNormal3f(mesh.normals[t.v2].x,
                       mesh.normals[t.v2].y,
                       mesh.normals[t.v2].z);
            glVertex3f(mesh.vertices[t.v2].x,
                       mesh.vertices[t.v2].y,
                       mesh.vertices[t.v2].z);
            glNormal3f(mesh.normals[t.v3].x,
                       mesh.normals[t.v3].y,
                       mesh.normals[t.v3].z);
            glVertex3f(mesh.vertices[t.v3].x,
                       mesh.vertices[t.v3].y,
                       mesh.vertices[t.v3].z);

            glNormal3f(mesh.normals[t.v3].x,
                       mesh.normals[t.v3].y,
                       mesh.normals[t.v3].z);
            glVertex3f(mesh.vertices[t.v3].x,
                       mesh.vertices[t.v3].y,
                       mesh.vertices[t.v3].z);
            glNormal3f(mesh.normals[t.v1].x,
                       mesh.normals[t.v1].y,
                       mesh.normals[t.v1].z);
            glVertex3f(mesh.vertices[t.v1].x,
                       mesh.vertices[t.v1].y,
                       mesh.vertices[t.v1].z);

        }
        glEnd();
    }
    if (_showVerts && !mesh.vertices.empty()) {
        glColor3f(0.9, 0.1, 0.1);
        glBegin(GL_POINTS);
        auto start = mesh.vertices.begin();
        auto end = mesh.vertices.end();
        auto it = mesh.vertices.begin();
        for (; it != end; ++it) {
            glNormal3f(mesh.normals[std::distance(start, it)].x,
                       mesh.normals[std::distance(start, it)].y,
                       mesh.normals[std::distance(start, it)].z);
            glVertex3f((*it).x, (*it).y, (*it).z);
        }
        glEnd();
    }
    if (_showNormals && !mesh.normals.empty()) {
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 0.0, 1.0);
        glBegin(GL_LINES);
        for (size_t i = 0; i < mesh.normals.size(); ++i) {
          glNormal3f(mesh.normals[i].x,
                     mesh.normals[i].y,
                     mesh.normals[i].z);
          glVertex3f((mesh.vertices[i].x + _NormalsLength * mesh.normals[i].x),
                     (mesh.vertices[i].y + _NormalsLength * mesh.normals[i].y),
                     (mesh.vertices[i].z + _NormalsLength * mesh.normals[i].z));
          glNormal3f(mesh.normals[i].x,
                     mesh.normals[i].y,
                     mesh.normals[i].z);
          glVertex3f(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
    glPopMatrix();
}

void GLWidget::draw_grid(Mesh& mesh) {

  Vertex max(0., 0., 0.);
  Vertex min(1., 1., 1.);

  for (const auto& v : mesh.vertices) {
    if (v.x >= max.x) max.x = v.x;
    if (v.y >= max.y) max.y = v.y;
    if (v.z >= max.z) max.z = v.z;
    if (v.x <= min.x) min.x = v.x;
    if (v.y <= min.y) min.y = v.y;
    if (v.z <= min.z) min.z = v.z;
  }

  /*
  glPushMatrix();
    glTranslatef(max.x, max.y, max.z);
    glColor3f(1.0, 0.0, 0.0);
    glutWireSphere(0.1, 10, 10);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(min.x, min.y, min.z);
    glColor3f(1.0, 0.0, 0.0);
    glutWireSphere(0.1, 10, 10);
  glPopMatrix();
  //*/

  int dim_x = ceil(fabs(max.x - min.x) / mesh.grid_size);
  int dim_y = ceil(fabs(max.y - min.y) / mesh.grid_size);
  int dim_z = ceil(fabs(max.z - min.z) / mesh.grid_size);

  //std::cout << "Grid size = " << dim_x << " * " << dim_y << " * " << dim_z << "\n";

  float displacement_x = min.x + mesh.grid_size / 2;
  float displacement_y = min.y + mesh.grid_size / 2;
  float displacement_z = min.z + mesh.grid_size / 2;

  glTranslatef(displacement_x, displacement_y, displacement_z);

  //*
  auto v_idx = mesh.voxel_grid.begin();
  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {
        if (*v_idx) {
          glPushMatrix();
            glColor3f((float)3  / grid_x,
                      (float)3  / grid_y,
                      (float)3  / grid_z);
            glTranslatef(grid_x * mesh.grid_size,
                         grid_y * mesh.grid_size,
                         grid_z * mesh.grid_size);
            glutWireCube(mesh.grid_size);
          glPopMatrix();
        }
        v_idx++;
      }
    }
  }
  //*/

  /*
  auto v_idx = vidx.begin();

  float radius = sqrt(pow(step_x / 2., 2) +
                      pow(step_y / 2., 2) +
                      pow(step_z / 2., 2));

  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {
        if (*v_idx) {
          glPushMatrix();
            glColor3f(1.0, 1.0, 0.0);
            //glColor3f(grid_x / dim_x, grid_y / dim_y, grid_z / dim_z);
            //std::cout << "Color is " << grid_x / (float)dim_x << " "
                      //<< grid_y / (float)dim_y << " "
                      //<< grid_z / (float)dim_z << "\n";
            glTranslatef(grid_x * step_x, grid_y * step_y, grid_z * step_z);
            glutWireSphere(radius, 15, 5);
          glPopMatrix();
        }
        v_idx++;
      }
    }
  }
  std::cout << "grid :: " << dim_x * dim_y * dim_z << "\n";
  //*/
}

void GLWidget::draw_axis() {

    glPushMatrix();
        glColor3f(sphere_color, 0.0, 0.0);
        glutSolidSphere(0.05, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.5, 0.0, 0.0);
    glEnd();

    glPushMatrix();
        glTranslatef(0.5, 0.0, 0.0);
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.5, 0.0);
    glEnd();

    glPushMatrix();
        glTranslatef(0.0, 0.5, 0.0);
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.5);
    glEnd();

    glPushMatrix();
        glTranslatef(0.0, 0.0, 0.5);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();
}
