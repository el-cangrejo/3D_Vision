#include "DBWidget.hpp"

#include <GL/glu.h>
#include <GL/glut.h>

DBWidget::DBWidget(QWidget *parent) : QGLWidget(parent) {}

void DBWidget::initializeGL() {
  glClearColor(_BackgroundColorR, _BackgroundColorG, _BackgroundColorB, 0.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  up_vector = Vertex(0, 1, 0);
  sphere = Vertex(-0.5, -0.5, -0.5);
}

void DBWidget::paintGL() {
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
	float grid_size = 1.;
	/*
	if (_showDatabase) {
		int total_num = meshes_to_render->size();
		int num_rows = std::ceil(std::sqrt(total_num));
		int num_cols = num_rows;
		
		std::cout << "Rows : " << num_rows << " Cols : " << num_cols << "\n";
		for (int i = 0; i < num_rows; ++i) {
			for (int j = 0; j < num_cols; ++j) {
				glPushMatrix();
				float trans_x = -std::floor(num_cols / 2.)  * grid_size + i * grid_size;
				float trans_y =  std::floor(num_rows / 2.) * grid_size - j * grid_size;

				//float trans_x = -3 + j * 1.5;
				//float trans_y = 3 - (i - 1) * 1.5;
				
				//std::cout << "X: " << trans_x << " Υ: " << trans_y << "\n";
				glTranslatef(trans_x, trans_y, 0);
				glScalef(0.5, 0.5, 0.5);
				if (i * num_cols + j < total_num) {
					std::cout << "Row : " << i << " Col : " << j << "\n";
					std::cout << "Num : " << i * num_cols + j << "\n";
					//std::cout << "Distance : " << meshes_to_render[(i - 1) * 5 + j].overall_distance << "\n";
					//glutWireSphere(1.0, 20, 20);
					draw_mesh((*meshes_to_render)[i * num_cols + j]);
				}
				glPopMatrix();
			}
			std::cout << "\n";
		}
		//std::cout << "\n";
	}
	//*/

	if (_showDatabase) {
			int total_num = meshes_to_render->size();
			int num_rows = std::ceil(total_num / 5.);
			//std::cout << "Rows : " << num_rows << "\n";
			for (int j = 0; j < 5; ++j) {
				for (int i = 1; i < num_rows + 1; ++i) {
					glPushMatrix();
					float trans_x = -3 + j * 1.5;
					float trans_y = 3 - (i - 1) * 1.5;
					//std::cout << "X: " << trans_x << " Υ: " << trans_y << "\n";
					glTranslatef(trans_x, trans_y, 0);
					glScalef(0.6, 0.6, 0.6);
					if ((i - 1) * 5 + j < total_num) {
						//std::cout << "Idx : " << (i - 1) * 5 + j << "\n";
						//std::cout << "Distance : " << meshes_to_render[(i - 1) * 5 + j].overall_distance << "\n";
						//glutWireSphere(1.0, 20, 20);
						draw_mesh((*meshes_to_render)[(i - 1) * 5 + j]);
					}
					glPopMatrix();
				}
			}
			//std::cout << "\n";
	}

  if (_showAxis) {
    glPushMatrix();
    glScalef(scale_size, scale_size, scale_size);
    draw_axis();
    glPopMatrix();
  }
}

void DBWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)w / h, 0.01, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void DBWidget::mousePressEvent(QMouseEvent *qevent) {
  if (qevent->button() == Qt::LeftButton) {
    mouseClickDown = true;
    mx0 = qevent->x();
    my0 = qevent->y();
  } else {
    mouseClickDown = false;
  }
}

void DBWidget::mouseMoveEvent(QMouseEvent *qevent) {
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

void DBWidget::wheelEvent(QWheelEvent *qevent) {
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

void DBWidget::keyPressEvent(QKeyEvent *qevent) {
    switch (qevent->key()) {
    case Qt::Key_Up: {
        targety += 0.1;
        break;
    }  case Qt::Key_Down: {
        targety -= 0.1;
        break;
    }
    case Qt::Key_Left: {
        targetx -= 0.1;
        break;
    }  case Qt::Key_Right: {
        targetx += 0.1;
        break;
    } default:
        break;
    }
    updateGL();
}

/*
 * Drawing Functions
 * */

void DBWidget::draw_mesh(Mesh &mesh) {
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

void DBWidget::draw_grid(Mesh &mesh) {
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

void DBWidget::draw_axis() {

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

void DBWidget::setModelColorR(float color) {
  _ModelColorR = color;
  updateGL();
}

void DBWidget::setModelColorB(float color) {
  _ModelColorB = color;
  updateGL();
}

void DBWidget::setModelColorG(float color) {
  _ModelColorG = color;
  updateGL();
}

void DBWidget::setBackgroundColorR(float color) {
  _BackgroundColorR = color;
  updateGL();
}

void DBWidget::setBackgroundColorG(float color) {
  _BackgroundColorG = color;
  updateGL();
}

void DBWidget::setBackgroundColorB(float color) {
  _BackgroundColorB = color;
  updateGL();
}

/*
 * Rendering Options Setting Functions
 * */

void DBWidget::setShowVertices(bool show) {
  _showVerts = show;
  updateGL();
}

void DBWidget::setShowTriangles(bool show) {
  _showTriangles = show;
  updateGL();
}

void DBWidget::setShowWire(bool show) {
  _showWire = show;
  updateGL();
}

void DBWidget::setShowNormals(bool show) {
  _showNormals = show;
  updateGL();
}

void DBWidget::setShowGrid(bool show) {
  _showGrid = show;
  updateGL();
}

void DBWidget::setShowSolid(bool show) {
  _showSolid = show;
  updateGL();
}

void DBWidget::setShowAxis(bool show) {
  _showAxis = show;
  updateGL();
}

void DBWidget::setShowFiltered(bool show) {
	if (show) {
		meshes_to_render = &processed_database_meshes;
	} else {
		meshes_to_render = &database_meshes;
	}
  updateGL();
}

void DBWidget::setShowQuery(bool show) {
	if (show) {
		meshes_to_render = &query_database_meshes;
	} else {
		meshes_to_render = &database_meshes;
	}
  updateGL();
}

void DBWidget::setModelLighting(bool light) {
  _modelLighting = light;
  updateGL();
}

void DBWidget::setNormalsLighting(bool light) {
  _normalLighting = light;
  updateGL();
}

void DBWidget::setShowDatabase(bool show) {
	_showDatabase = show;
	updateGL();
}
