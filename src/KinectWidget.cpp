#include "KinectWidget.hpp"

KinectWidget::KinectWidget(QWidget *parent)
    : QGLWidget(parent), kinect_initialized(false),
      depthMat(cv::Size(640, 480), CV_16UC1), listener(libfreenect2::Frame::Color |
				 	libfreenect2::Frame::Ir | libfreenect2::Frame::Depth) {
}

void KinectWidget::initializeGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);

  glGenTextures(1, &gl_depth_tex);
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);


	glGenTextures(1, &gl_rgb_tex);
  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

void KinectWidget::paintGL() {
  //static std::vector<uint8_t> depth(640 * 480 * 4, 0);
  //static std::vector<uint8_t> rgb(640 * 480 * 4, 0);

	libfreenect2::Frame *rgb_ = nullptr;
	libfreenect2::Frame *ir_ = nullptr;
	libfreenect2::Frame *depth_ = nullptr;

  if (kinect_initialized) {

    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "timeout!" << std::endl;
			kinect_initialized = !kinect_initialized;
			return;
    }

    rgb_ = frames[libfreenect2::Frame::Color];
    ir_ = frames[libfreenect2::Frame::Ir];
    depth_ = frames[libfreenect2::Frame::Depth];
		
  } else {
		return;
	}

    // wipe the drawing surface clear
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	
	libfreenect2::Frame undistorted(512, 424, 4);

	registration->undistortDepth(depth_, &undistorted);
	int depth_size = depth_->width * depth_->height * 4;
	unsigned char * depth_data = new unsigned char[depth_size];
	std::copy(depth_->data, depth_->data + depth_->width * depth_->height * 4, depth_data);
	//std::copy(undistorted.data, undistorted.data + depth_size, depth_data);
	//std::copy(ir_->data, ir_->data + depth_size, depth_data);

	//std::for_each(&depth_data[0], &depth_data[depth_size], [](unsigned char &x) { x /= 500;});

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 512, 424, 0, GL_RED, GL_FLOAT,
               depth_data);

	glBindTexture(GL_TEXTURE0, gl_depth_tex);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 512, 424, GL_RED, GL_FLOAT, depth_data);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(width_, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(width_, height_ / 2.0, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, height_ / 2.0, 0);
  glEnd();

	int rgb_size = rgb_->width * rgb_->height * 4;
	unsigned char * rgb_data = new unsigned char[rgb_size];
	std::copy(rgb_->data, rgb_->data + rgb_->width * rgb_->height * 4, rgb_data);


  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgb_->width, rgb_->height, 0, GL_BGRA, GL_UNSIGNED_BYTE,
               rgb_data);

	glBindTexture(GL_TEXTURE0, gl_rgb_tex);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, rgb_->width, rgb_->height, GL_BGRA, GL_UNSIGNED_BYTE, rgb_data);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, height_ / 2.0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(width_, height_ / 2.0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(width_, height_, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, height_, 0);
  glEnd();
	listener.release(frames);
	delete[] depth_data;
	delete[] rgb_data;
}

void KinectWidget::resizeGL(int w, int h) {
  width_ = w;
  height_ = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, w, h, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

Mesh KinectWidget::generateMesh() {
 	Mesh m; 
	if (!listener.waitForNewFrame(frames, 10*1000)) return m; // 10 sconds
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	
	libfreenect2::Frame undistorted(512, 424, 4);
  libfreenect2::Frame *depth_ = frames[libfreenect2::Frame::Depth];

	registration->undistortDepth(depth_, &undistorted);


	for (int i = 0; i < 512; ++i) {
		for (int j = 0; j < 424; ++j) {
			Vertex v(0, 0, 0);
			float x = 0, y = 0, z = 0;
			registration->getPointXYZ(&undistorted, j, i, x, y, z);
			//std::cout << "Generate (x, y , z) : " << x << " " << y << " " << z << "\n";
			if (std::isnan(x)) continue;
			v.x = x;
			v.y = -y;
			v.z = z;
			m.vertices.push_back(v);
		}
	}
	delete registration;
	listener.release(frames);
	return m;
}

void KinectWidget::timerEvent(QTimerEvent *event) { updateGL(); }

void KinectWidget::addFrame(std::string id, libfreenect2::Frame *frame) {
	frames_[id] = frame;
}
