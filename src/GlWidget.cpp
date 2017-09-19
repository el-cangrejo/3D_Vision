#include "GlWidget.hpp"

#include <GL/glu.h>
#include <GL/glut.h>

GLWidget::GLWidget(QWidget *parent) : QGLWidget(parent) {}

void GLWidget::initializeGL() {
  glClearColor(_BackgroundColorR, _BackgroundColorG, _BackgroundColorB, 0.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  up_vector = Vertex(0, 1, 0);
  sphere = Vertex(-0.5, -0.5, -0.5);
}

void GLWidget::paintGL() {
  if (_modelLighting)
    glEnable(GL_LIGHTING);
  else
    glDisable(GL_LIGHTING);

  glClearColor(_BackgroundColorR, _BackgroundColorG, _BackgroundColorB, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(eyex, eyey, eyez, // camera position
            targetx, targety, targetz,          // target position
            up_vector.x, up_vector.y, up_vector.z);

  float scale_size(1 / 1.5);

//  {
//  }
//  if (_showFilteredMesh && !filtered_mesh.empty()) {
//    glPushMatrix();
//    glTranslatef(-0.6, 0, 0);
//    glScalef(scale_size, scale_size, scale_size);
//    draw_mesh(filtered_mesh);
//    glPopMatrix();
//  }
//  if (_showTargetMesh && !target_mesh.empty()) {
//    glPushMatrix();
//    glTranslatef(-0.6, 0.0, 0.0);
//    glScalef(scale_size, scale_size, scale_size);
//    draw_mesh(target_mesh);
//    glPopMatrix();
//  }
	if (_showMultiMesh) {
		int total_num = primary_meshes.size();
		if (total_num < 5) {
			for (int i = 1; i < total_num; ++i) {
				glPushMatrix();
				float trans_x = -0.5 + 0.1 + i * 1/ 5.;
				//float trans_y = -0.5 + 1 / static_cast<float>(num_rows) / 2. + i * 1 / static_cast<float>(num_rows);
				glTranslatef(trans_x, 0, 0);
				glScalef(0.3, 0.3, 0.3);
				draw_mesh(primary_meshes[i]);
				glPopMatrix();
			}
		} else {
			int num_rows = std::ceil(total_num / 5);
			for (int j = 0; j < 5; ++j) {
				for (int i = 0; i < num_rows; ++i) {
					glPushMatrix();
					float trans_x = -0.5 + 0.1 + j * 1/ 5.;
					float trans_y = -0.5 + 1 / static_cast<float>(num_rows) / 2. + i * 1 / static_cast<float>(num_rows);
					glTranslatef(trans_x, trans_y, 0);
					glScalef(0.3, 0.3, 0.3);
					if (i * 5 + j <= total_num)
						draw_mesh(primary_meshes[i * 5 + j + 1]);
					glPopMatrix();
				}
			}
		}
	} else {
    glPushMatrix();
    if ((_showFilteredMesh && !filtered_mesh.empty()) ||
        (_showTargetMesh && !target_mesh.empty())) {
      glTranslatef(0.6, 0, 0);
      glScalef(scale_size, scale_size, scale_size);
    }
		if (!primary_meshes.empty())
			draw_mesh(primary_meshes[0]);
    if (_showGrid)
      draw_grid(primary_mesh);
    glPopMatrix();
	}

  if (_showAxis) {
    glPushMatrix();
   // glTranslatef(-0.9, -0.9, -0.9);
    glScalef(scale_size, scale_size, scale_size);
    draw_axis();
    glPopMatrix();
  }
}

void GLWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)w / h, 0.01, 100);
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
    azimuthAngle -= (qevent->x() - mx0) * _rotFactor;
    altitudeAngle -= (qevent->y() - my0) * _rotFactor;
    // Set new camrea position
    eyex = cdist * sin(altitudeAngle) * sin(azimuthAngle);
    eyey = cdist * cos(altitudeAngle);
    eyez = cdist * sin(altitudeAngle) * cos(azimuthAngle);

    if ((altitudeAngle > 2 * PI) || (altitudeAngle < -2 * PI))
      altitudeAngle = 0.0;

    if (altitudeAngle < -PI) {
      up_vector.y = 1;
    } else if (altitudeAngle < 0) {
      up_vector.y = -1;
    } else if (altitudeAngle < PI) {
      up_vector.y = 1;
    } else if (altitudeAngle > PI) {
      up_vector.y = -1;
    }

    // Keep mouse x,y for next call
    mx0 = qevent->x();
    my0 = qevent->y();
    updateGL();
  }
}

void GLWidget::wheelEvent(QWheelEvent *qevent) {
  if (qevent->delta() > 0) {
    cdist -= _zoomStep;
  } else if (qevent->delta() < 0) {
    cdist += _zoomStep;
  }
  eyex = cdist * sin(altitudeAngle) * sin(azimuthAngle);
  eyey = cdist * cos(altitudeAngle);
  eyez = cdist * sin(altitudeAngle) * cos(azimuthAngle);
  updateGL();
}

void GLWidget::keyPressEvent(QKeyEvent *qevent) {
    switch (qevent->key()) {
    case Qt::Key_Up: {
        targetx += 0.1;
        break;
    }  case Qt::Key_Down: {
        targetx -= 0.1;
        break;
    }
    case Qt::Key_Left: {
        targety -= 0.1;
        break;
    }  case Qt::Key_Right: {
        targety += 0.1;
        break;
    } default:
        break;
    }
    updateGL();
}

/*
 * Drawing Functions
 * */

void GLWidget::draw_mesh(Mesh &mesh) {
  glPushMatrix();
  if (_showTriangles && !mesh.triangles.empty()) {
    if (!_showSolid)
      glDisable(GL_DEPTH_TEST);
    else
      glEnable(GL_DEPTH_TEST);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_SMOOTH);
    glBegin(GL_TRIANGLES);
    glColor3f(_ModelColorR, _ModelColorG, _ModelColorB);
    for (const auto &t : mesh.triangles) {
			//std::cout << "Triangle " << t.v1 << " " << t.v2 << " " << t.v3 << "\n";
      // glColor3f(0.9, 0.1, 0.1);
      glNormal3f(mesh.normals[t.v1].x, mesh.normals[t.v1].y,
                 mesh.normals[t.v1].z);
      glVertex3f(mesh.vertices[t.v1].x, mesh.vertices[t.v1].y,
                 mesh.vertices[t.v1].z);
      // glColor3f(0.1, 0.9, 0.1);
      glNormal3f(mesh.normals[t.v2].x, mesh.normals[t.v2].y,
                 mesh.normals[t.v2].z);
      glVertex3f(mesh.vertices[t.v2].x, mesh.vertices[t.v2].y,
                 mesh.vertices[t.v2].z);
      // glColor3f(0.1, 0.1, 0.9);
      glNormal3f(mesh.normals[t.v3].x, mesh.normals[t.v3].y,
                 mesh.normals[t.v3].z);
      glVertex3f(mesh.vertices[t.v3].x, mesh.vertices[t.v3].y,
                 mesh.vertices[t.v3].z);
    }
    glEnd();
  }
  if (_showWire && !mesh.triangles.empty()) {
    glBegin(GL_LINES);
    glColor3f(0.8, 0.0, 0.0);
    for (const auto &t : mesh.triangles) {
      glNormal3f(mesh.normals[t.v1].x, mesh.normals[t.v1].y,
                 mesh.normals[t.v1].z);
      glVertex3f(mesh.vertices[t.v1].x, mesh.vertices[t.v1].y,
                 mesh.vertices[t.v1].z);
      glNormal3f(mesh.normals[t.v2].x, mesh.normals[t.v2].y,
                 mesh.normals[t.v2].z);
      glVertex3f(mesh.vertices[t.v2].x, mesh.vertices[t.v2].y,
                 mesh.vertices[t.v2].z);

      glNormal3f(mesh.normals[t.v2].x, mesh.normals[t.v2].y,
                 mesh.normals[t.v2].z);
      glVertex3f(mesh.vertices[t.v2].x, mesh.vertices[t.v2].y,
                 mesh.vertices[t.v2].z);
      glNormal3f(mesh.normals[t.v3].x, mesh.normals[t.v3].y,
                 mesh.normals[t.v3].z);
      glVertex3f(mesh.vertices[t.v3].x, mesh.vertices[t.v3].y,
                 mesh.vertices[t.v3].z);

      glNormal3f(mesh.normals[t.v3].x, mesh.normals[t.v3].y,
                 mesh.normals[t.v3].z);
      glVertex3f(mesh.vertices[t.v3].x, mesh.vertices[t.v3].y,
                 mesh.vertices[t.v3].z);
      glNormal3f(mesh.normals[t.v1].x, mesh.normals[t.v1].y,
                 mesh.normals[t.v1].z);
      glVertex3f(mesh.vertices[t.v1].x, mesh.vertices[t.v1].y,
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
      if (!mesh.normals.empty()) {
        glNormal3f(mesh.normals[std::distance(start, it)].x,
                   mesh.normals[std::distance(start, it)].y,
                   mesh.normals[std::distance(start, it)].z);
      }
      if (!mesh.colors.empty()) {
        auto v = mesh.colors[std::distance(start, it)];
        glColor3f(v.val[0] / 255.,
                  v.val[1] / 255.,
                  v.val[2] / 255.);
      }
      glVertex3f((*it).x, (*it).y, (*it).z);
			//std::cout << "Vertex " << (*it).x << " " << (*it).y << " " << (*it).z << "\n";

    }
    glEnd();
  }
  if (_showNormals && !mesh.normals.empty()) {
    if (!_normalLighting)
      glDisable(GL_LIGHTING);
    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (size_t i = 0; i < mesh.normals.size(); ++i) {
      glNormal3f(mesh.normals[i].x, mesh.normals[i].y, mesh.normals[i].z);
      glVertex3f((mesh.vertices[i].x + _NormalsLength * mesh.normals[i].x),
                 (mesh.vertices[i].y + _NormalsLength * mesh.normals[i].y),
                 (mesh.vertices[i].z + _NormalsLength * mesh.normals[i].z));
      glNormal3f(mesh.normals[i].x, mesh.normals[i].y, mesh.normals[i].z);
      glVertex3f(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
    }
    glEnd();
    if (!_normalLighting)
      glEnable(GL_LIGHTING);
  }
  glPopMatrix();
}

void GLWidget::draw_grid(Mesh &mesh) {
  int dim_x = ceil(fabs(mesh.max.x - mesh.min.x) / mesh.grid_size);
  int dim_y = ceil(fabs(mesh.max.y - mesh.min.y) / mesh.grid_size);
  int dim_z = ceil(fabs(mesh.max.z - mesh.min.z) / mesh.grid_size);

  // std::cout << "Grid size = " << dim_x << " * " << dim_y << " * " << dim_z <<
  // "\n";

  float displacement_x = mesh.min.x + mesh.grid_size / 2;
  float displacement_y = mesh.min.y + mesh.grid_size / 2;
  float displacement_z = mesh.min.z + mesh.grid_size / 2;

  glTranslatef(displacement_x, displacement_y, displacement_z);

  //*
  auto v_idx = mesh.voxel_grid.begin();
  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {
        if (*v_idx) {
          glPushMatrix();
          glColor3f((float)3 / grid_x, (float)3 / grid_y, (float)3 / grid_z);
          glTranslatef(grid_x * mesh.grid_size, grid_y * mesh.grid_size,
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

/*
 * Color Setting Functions
 * */

void GLWidget::setModelColorR(float color) {
  _ModelColorR = color;
  updateGL();
}

void GLWidget::setModelColorB(float color) {
  _ModelColorB = color;
  updateGL();
}

void GLWidget::setModelColorG(float color) {
  _ModelColorG = color;
  updateGL();
}

void GLWidget::setBackgroundColorR(float color) {
  _BackgroundColorR = color;
  updateGL();
}

void GLWidget::setBackgroundColorG(float color) {
  _BackgroundColorG = color;
  updateGL();
}

void GLWidget::setBackgroundColorB(float color) {
  _BackgroundColorB = color;
  updateGL();
}

/*
 * Rendering Options Setting Functions
 * */

void GLWidget::setShowVertices(bool show) {
  _showVerts = show;
  updateGL();
}

void GLWidget::setShowTriangles(bool show) {
  _showTriangles = show;
  updateGL();
}

void GLWidget::setShowWire(bool show) {
  _showWire = show;
  updateGL();
}

void GLWidget::setShowNormals(bool show) {
  _showNormals = show;
  updateGL();
}

void GLWidget::setShowGrid(bool show) {
  _showGrid = show;
  updateGL();
}

void GLWidget::setShowSolid(bool show) {
  _showSolid = show;
  updateGL();
}

void GLWidget::setShowAxis(bool show) {
  _showAxis = show;
  updateGL();
}

void GLWidget::setShowFilteredMesh(bool show) {
  _showFilteredMesh = show;
  updateGL();
}

void GLWidget::setShowTargetMesh(bool show) {
  _showTargetMesh = show;
  updateGL();
}

void GLWidget::setModelLighting(bool light) {
  _modelLighting = light;
  updateGL();
}

void GLWidget::setNormalsLighting(bool light) {
  _normalLighting = light;
  updateGL();
}

void GLWidget::setMultiMesh(bool show) {
	_showMultiMesh = show;
	updateGL();
}

/*
 * Database Handling Functions
 * */

void GLWidget::eraseDatabase() {
  std::cout << "Erasing contents of the database.. \n";
  db_descriptors.clear();
  db_descriptors.shrink_to_fit();

  db_files.clear();
  db_files.shrink_to_fit();

  database.clear();
  database.shrink_to_fit();
  std::cout << "Database is now empty!\n";
}
