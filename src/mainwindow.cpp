#include "mainwindow.h"
#include "Segmentation.hpp"
#include "GlWidget.hpp"
#include "ui_mainwindow.h"
#include "Database.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>

#include <libfreenect2/libfreenect2.hpp>

#include <QFileDialog>
#include <QUrl>
#include <boost/filesystem.hpp>
#include <thread>

extern Mesh filtered_mesh;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  inititializeUI();
  connectUI();
}

MainWindow::~MainWindow() { delete ui; }

/*
 *  Initialize Functions
 * */

void MainWindow::dragEnterEvent(QDragEnterEvent *e) {
    if (e->mimeData()->hasUrls()) {
        e->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent *e) {
    foreach (const QUrl &url, e->mimeData()->urls()) {
        QString fileName = url.toLocalFile();
				std::cout << "Dropped file:" << fileName.toStdString() << "\n";
				if (fileName.endsWith("obj") || fileName.endsWith("off")) {
					OpenMesh(fileName);
				} else {
					OpenImage(fileName, CV_16UC1);
				}
		}
}

void MainWindow::inititializeUI() {
	setAcceptDrops(true);
  // CheckBox Initializers
  ui->ShowTrianglesCheckBox->setChecked(ui->widget->_showTriangles);
  ui->ShowSolidCheckBox->setChecked(ui->widget->_showSolid);
  ui->ShowAxisCheckBox->setChecked(ui->widget->_showAxis);
  ui->ShowVerticesCheckBox->setChecked(ui->widget->_showVerts);
  ui->ShowNormalsCheckBox->setChecked(ui->widget->_showNormals);
  ui->ShowWireCheckBox->setChecked(ui->widget->_showWire);
	ui->MultiMeshCheckBox->setChecked(ui->widget->_showMultiMesh);

  // Sliders Initializers
  ui->ModelColorBSlider->setSliderPosition(
      (int)std::floor(ui->widget->_ModelColorB * 99.));
  ui->ModelColorRSlider->setSliderPosition(
      (int)std::floor(ui->widget->_ModelColorR * 99.));
  ui->ModelColorGSlider->setSliderPosition(
      (int)std::floor(ui->widget->_ModelColorG * 99.));

  ui->BackgroundColorRSlider->setSliderPosition(ui->widget->_BackgroundColorR *
                                                99.);
  ui->BackgroundColorGSlider->setSliderPosition(ui->widget->_BackgroundColorB *
                                                99.);
  ui->BackgroundColorBSlider->setSliderPosition(ui->widget->_BackgroundColorG *
                                                99.);

  ui->RotFactorSlider->setSliderPosition(ui->widget->_rotFactor * 999.);
  ui->ZoomFactSlider->setSliderPosition(ui->widget->_zoomStep * 799.);
}

void MainWindow::connectUI() {
  // Connect Actions
  connect(ui->action_OpenMesh, SIGNAL(triggered()), this,
          SLOT(onActionOpenMesh()));
  connect(ui->action_OpenImage8U, SIGNAL(triggered()), this,
          SLOT(onActionOpenImage8U()));
  connect(ui->action_OpenImage16U, SIGNAL(triggered()), this,
          SLOT(onActionOpenImage16U()));
  connect(ui->action_OpenKinect, SIGNAL(triggered()), this,
          SLOT(onActionOpenKinect()));
  connect(ui->action_Quit, SIGNAL(triggered()), this, SLOT(onActionQuit()));
	
	// Database Actions
  connect(ui->action_Load_Database, SIGNAL(triggered()), this,
          SLOT(onActionLoadDatabase()));
  connect(ui->action_Preprocess_Database, SIGNAL(triggered()), this,
          SLOT(onActionPreprocessDatabase()));
  connect(ui->action_Load_Classes, SIGNAL(triggered()), this,
          SLOT(onActionLoadClasses()));

  connect(ui->action_TakeSnapshot, SIGNAL(triggered()), this,
          SLOT(onActionTakeSnapshot()));

  // Connect Sliders
  connect(ui->ModelColorRSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setModelColorR(int)));
  connect(ui->ModelColorGSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setModelColorG(int)));
  connect(ui->ModelColorBSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setModelColorB(int)));

  connect(ui->BackgroundColorRSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setBackgroundColorR(int)));
  connect(ui->BackgroundColorGSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setBackgroundColorG(int)));
  connect(ui->BackgroundColorBSlider, SIGNAL(valueChanged(int)), this,
          SLOT(setBackgroundColorB(int)));

  // Connect CheckBoxes
  connect(ui->ShowVerticesCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setShowVertices(bool)));
  connect(ui->ShowTrianglesCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setShowTriangles(bool)));
  connect(ui->ShowWireCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setShowWire(bool)));
  connect(ui->ShowNormalsCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setShowNormals(bool)));
  connect(ui->ShowGridFilter, SIGNAL(clicked(bool)), this,
          SLOT(setShowGrid(bool)));
  connect(ui->ShowAxisCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setShowAxis(bool)));
  connect(ui->ShowFilteredMesh, SIGNAL(clicked(bool)), this,
          SLOT(setShowFilteredMesh(bool)));
  connect(ui->ModelLightingCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setModelLighting(bool)));
  connect(ui->NormalsLightingCheckBox, SIGNAL(clicked(bool)), this,
          SLOT(setNormalsLighting(bool)));
  connect(ui->ShowSolidCheckBox, SIGNAL(clicked(bool)), this,
            SLOT(setShowSolid(bool)));
	connect(ui->MultiMeshCheckBox, SIGNAL(clicked(bool)), this, 
						SLOT(setMultiMesh(bool)));
	connect(ui->ShowDatabaseCheckBox, SIGNAL(clicked(bool)), this, 
						SLOT(setShowDatabase(bool)));
  connect(ui->ShowDBVertices, SIGNAL(clicked(bool)), this,
          SLOT(setShowDBVertices(bool)));
  connect(ui->ShowDBTriangles, SIGNAL(clicked(bool)), this,
          SLOT(setShowDBTriangles(bool)));
  connect(ui->ShowDBFiltered, SIGNAL(clicked(bool)), this,
          SLOT(setShowDBFiltered(bool)));

  // Connect Buttons
  connect(ui->GridFilter, SIGNAL(clicked()), this, SLOT(gridFilter()));
  connect(ui->SegmentatImgButton, SIGNAL(clicked()), this,
          SLOT(onSegmentImg()));
  connect(ui->GenerateMeshButton, SIGNAL(clicked()), this, SLOT(onGenerateMesh()));
  connect(ui->DrawButton, SIGNAL(clicked()), this, SLOT(onDraw()));
  connect(ui->ClearAllButton, SIGNAL(clicked()), this, SLOT(onClearAll()));
  connect(ui->ZoomImgWidgetButton, SIGNAL(clicked()), this, SLOT(onZoomImgWidget()));
  connect(ui->ComputeDescriptorsButton, SIGNAL(clicked()), this, SLOT(onComputeDescriptors()));
  connect(ui->RetrievalButton, SIGNAL(clicked()), this, SLOT(onRetrieve()));
  connect(ui->DeleteDB, SIGNAL(clicked()), this, SLOT(onDeleteDB()));
  connect(ui->PartialViewButton, SIGNAL(clicked()), this, SLOT(onPartialView()));
  connect(ui->BilateralFilterButton, SIGNAL(clicked()), this, SLOT(onBilateralFilter()));
  connect(ui->MultilateralFilterButton, SIGNAL(clicked()), this, SLOT(onMultilateralFilter()));
  
  // Segmenation
  connect(ui->median_kernelSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setMedianKernel(int)));
  connect(ui->normal_radiusSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setNormalsRadius(int)));
  connect(ui->edge_radiusSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setEdgeRadius(int)));
  connect(ui->KernelRadiusSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setKernelRadius(int)));

	// Database
  connect(ui->GridSizeDB, SIGNAL(valueChanged(double)), this, SLOT(setGridSizeDB(double)));

	// Mesh operations
	connect(ui->S_dSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSdBF(double)));
	connect(ui->S_nSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSnBF(double)));

	ui->GridSize->setDecimals(4);

	connect(ui->ChoosePointSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setChosenVertex(int)));
}

void MainWindow::setChosenVertex(int v) {
	ui->widget->chosen = v;
	ui->widget->vertex_chosen = true;
	ui->widget->updateGL();

	ui->widget->primary_meshes[0].printSignatureHistogram(v);
}

void MainWindow::on_StatOutFIlter_clicked() {
  ui->widget->primary_meshes[0] = ui->widget->primary_meshes[0].statoutFilter();
  ui->widget->primary_meshes[0].preprocess(); 
	ui->widget->primary_meshes[0].computeNormals();
	ui->widget->primary_meshes[0].triangulate();
	//ui->widget->primary_meshes[0].computeNormals();
	//ui->widget->primary_meshes[0].bilateralFilter();
	ui->widget->primary_meshes[0].printInfo();

  ui->widget->updateGL();
}

void MainWindow::on_GridSize_valueChanged(double arg1) {
  ui->widget->primary_meshes[0].setGridSize(arg1);
}

void MainWindow::on_NormalsLengthSlider_valueChanged(int value) {
  ui->widget->_NormalsLength = value / 999.;
  ui->widget->updateGL();
}

void MainWindow::on_ZoomFactSlider_sliderMoved(int position) {
  ui->widget->_zoomStep = position / 99.;
}

void MainWindow::on_RotFactorSlider_sliderMoved(int position) {
  ui->widget->_rotFactor = position / 999.;
}

void MainWindow::onComputeDescriptors() {
	if (ui->widget->primary_meshes.empty()) {
		std::cout << "\e[1;31mNo mesh loaded!\nPlease load mesh first and then try to compute desriptors!\e[0m\n";
		return;
	}

	std::cout << "Begin computing descriptors!\n";

	if (!ui->widget->filtered_mesh.empty()) {
		std::cout << "Using filtered mesh!\n";
		ui->widget->filtered_mesh.queryProcess();
		ui->widget->primary_meshes[0].queryProcess();
	} else {
		std::cout << "Using primary mesh!\n";
		ui->widget->primary_meshes[0].queryProcess();
	}
}

void MainWindow::onRetrieve() {
	if (ui->widget->filtered_mesh.empty()) {
		std::cout << "\e[1;31mNo mesh loaded for query!\e[0m\n";
		return;
	}
	if (ui->dbwidget->database_meshes.empty()) {
		std::cout << "\e[1;31mNo database loaded to retrieve from!\e[0m\n";
		return;
	}

	std::cout << "\e[1;32mBegin searching in DB!\n";
	
  clock_t begin, end;
  double elapsed_secs;
	
  begin = clock();
	for (int i = 0; i < ui->dbwidget->processed_database_meshes.size(); ++i)
		float dist = ui->widget->filtered_mesh.distanceTo(ui->dbwidget->processed_database_meshes[i]);
	end = clock();
  double elapsed_secs_s = double(end - begin) / CLOCKS_PER_SEC;

	for (int i = 0; i < ui->dbwidget->processed_database_meshes.size(); ++i) {
		ui->dbwidget->database_meshes[i].overall_distance = 
			ui->dbwidget->processed_database_meshes[i].overall_distance;
	}
	
	std::sort(ui->dbwidget->processed_database_meshes.begin(), ui->dbwidget->processed_database_meshes.end(), 
			[](const Mesh &a, const Mesh &b) -> bool { 
			return a.overall_distance < b.overall_distance;});

	std::sort(ui->dbwidget->database_meshes.begin(), ui->dbwidget->database_meshes.end(), 
			[](const Mesh &a, const Mesh &b) -> bool { 
			return a.overall_distance < b.overall_distance;});
	
	for (const auto &m : ui->dbwidget->database_meshes) 
		std::cout << "Distance :" << m.overall_distance << "\n";

	ui->dbwidget->updateGL();
}

void MainWindow::onDeleteDB() {
	if (!ui->dbwidget->database_meshes.empty()) {
		ui->dbwidget->database_meshes.clear();
		ui->dbwidget->database_meshes.shrink_to_fit();
	}
	if (!ui->dbwidget->processed_database_meshes.empty()) {
		ui->dbwidget->processed_database_meshes.clear();
		ui->dbwidget->processed_database_meshes.shrink_to_fit();
	}
	ui->dbwidget->updateGL();
}

void MainWindow::onPartialView() {
	takePartialView(ui->dbwidget->database_meshes, 0.33);
}

/*
 * Menu Actions
 * */

void MainWindow::onActionQuit() {
  std::cout << "User evoked exiting.. \nGoodbye!\n";
  QApplication::quit();
}

void MainWindow::OpenMesh(QString fileName) {
  if (!fileName.isEmpty()) {
		Mesh m;
		if (m.load(fileName.toStdString()) != 0) {
			m.preprocess();
			m.computeNormals();
			ui->NumOfPointsLabel->setText(QString::fromStdString("Num of Points : " + std::to_string(m.vertices.size())));
			ui->ChoosePointSpinBox->setMaximum(m.vertices.size());
			ui->widget->primary_meshes.clear();
			ui->widget->primary_meshes.push_back(m);
			//std::cout << "Primary meshes : " << ui->widget->primary_meshes.size() << "\n";
			//std::cout << "Primary mesh  triangle std : " << ui->widget->primary_meshes[0].trianglesAreaStd() << "\n";
			//ui->widget->primary_meshes[0].resample();
			ui->widget->updateGL();
		}
  } else {
		std::cout << "File name is empty\n";
	}
}

void MainWindow::onActionOpenMesh() {
  QString fileName = QFileDialog::getOpenFileName(this);
	OpenMesh(fileName);
}

void MainWindow::onActionOpenImage8U() {
  QString fileName = QFileDialog::getOpenFileName(this);
  OpenImage(fileName, CV_8UC1);
}

void MainWindow::onActionOpenImage16U() {
  QString fileName = QFileDialog::getOpenFileName(this);
  OpenImage(fileName, CV_16UC1);
}

void MainWindow::OpenImage(QString fileName, int type) {
  cv::Mat image;
  if (!fileName.isEmpty() && fileName.endsWith(".png")) {
		std::cout << "Opening image : " << fileName.toStdString() << "\n";
    if (type == CV_8UC1) {
        image = cv::imread(fileName.toStdString(), CV_8UC1);
        std::cout << "Number of channels: " << image.channels() << ", type: CV_8UC1\n";
    } else {
        image = cv::imread(fileName.toStdString(), CV_16UC1);
        std::cout << "Number of channels: " << image.channels() << ", type: CV_16UC1\n";
    }
  }
  ui->SnapShot_Widget->setImg(std::move(image));
}

void MainWindow::onActionOpenKinect() {
	if (ui->Kinect_Widget->kinect_initialized) {
		ui->Kinect_Widget->dev->stop();
		ui->Kinect_Widget->dev->close();
		ui->Kinect_Widget->kinect_initialized = !ui->Kinect_Widget->kinect_initialized;
		ui->action_OpenKinect->setText("Open &Kinect");
		return;
	}
/// [discovery]
  if(ui->Kinect_Widget->freenect2.enumerateDevices() == 0) {
    std::cout << "No device connected!" << std::endl;
    return;
  }
  if (ui->Kinect_Widget->serial == "") {
    ui->Kinect_Widget->serial = ui->Kinect_Widget->freenect2.getDefaultDeviceSerialNumber();
  }
/// [discovery]

  if(ui->Kinect_Widget->pipeline) {
/// [open]
    ui->Kinect_Widget->dev = ui->Kinect_Widget->freenect2.openDevice(ui->Kinect_Widget->serial, ui->Kinect_Widget->pipeline);
/// [open]
  } else {
    ui->Kinect_Widget->dev = ui->Kinect_Widget->freenect2.openDevice(ui->Kinect_Widget->serial);
  }

  if(ui->Kinect_Widget->dev == nullptr) {
    std::cout << "Failure opening device!" << std::endl;
    return;
  }

/// [listeners]
  ui->Kinect_Widget->dev->setColorFrameListener(&ui->Kinect_Widget->listener);
  ui->Kinect_Widget->dev->setIrAndDepthFrameListener(&ui->Kinect_Widget->listener);
/// [listeners]

/// [start]
  if (ui->Kinect_Widget->enable_rgb && ui->Kinect_Widget->enable_depth) {
    if (!ui->Kinect_Widget->dev->start())
      return;
  } else {
    if (!ui->Kinect_Widget->dev->startStreams(ui->Kinect_Widget->enable_rgb, ui->Kinect_Widget->enable_depth))
      return;
  }

  std::cout << "device serial: " << ui->Kinect_Widget->dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << ui->Kinect_Widget->dev->getFirmwareVersion() << std::endl;
/// [start]

  ui->Kinect_Widget->startTimer(10);
  ui->Kinect_Widget->kinect_initialized = true;
  ui->action_OpenKinect->setText("Close &Kinect");
}

void MainWindow::onActionLoadDatabase() {
  QString directory_name = QFileDialog::getExistingDirectory(this);
  if (!directory_name.isEmpty()) {
    std::cout << "Loading for display database : " << directory_name.toStdString() << "\n";
		loadDB(directory_name.toStdString(), ui->dbwidget->database_meshes);
		ui->dbwidget->updateGL();
  }
}

void MainWindow::onActionPreprocessDatabase() {
	if (ui->dbwidget->database_meshes.empty()) {
		std::cout << "\e[1;31mNo Database meshes loaded!\nPlease use action Load Database and try again!\e[0m\n";
		return;
	}

	preprocessDB(ui->dbwidget->database_meshes, ui->dbwidget->processed_database_meshes);
}

void MainWindow::onActionLoadClasses() {
  QStringList file_name = QFileDialog::getOpenFileNames(this);
  if (file_name.isEmpty()) {
		std::cout << "Empty file!\nDatabase classes not loaded!\n";
		return;
	}

	std::cout << "Loading database classes!\n";
	readObjectClasses(file_name.at(0).toStdString(), ui->dbwidget->database_meshes);
}

void MainWindow::onActionTakeSnapshot() {
  if (!ui->Kinect_Widget->kinect_initialized) {
    std::cout << "No Kinect device initialized!\n";
    return;
  }

  std::cout << "Taking snapshot.. \n";

	ui->Kinect_Widget->takeSnapshot();

  std::string filename("snapshot");
  std::string suffix(".png");
  std::ostringstream file;
  file << filename << suffix;
  imwrite(file.str(), ui->Kinect_Widget->depthMat);
  std::cout << "Took snapshot.. \n";
}

/*
 * Color Setting Functions
 * */

void MainWindow::setModelColorR(int color) {
  float RColor = color / 99.;
	//std::cout << "R: " << RColor << "\n";
  ui->widget->setModelColorR(RColor);
}

void MainWindow::setModelColorG(int color) {
  float GColor = color / 99.;
	//std::cout << "G: " << GColor << "\n";
  ui->widget->setModelColorG(GColor);
}

void MainWindow::setModelColorB(int color) {
  float BColor = color / 99.;
	//std::cout << "B: " << BColor << "\n";
  ui->widget->setModelColorB(BColor);
}

void MainWindow::setBackgroundColorR(int color) {
  float RColor = color / 99.;
	//std::cout << "R: " << RColor << "\n";
  ui->widget->setBackgroundColorR(RColor);
}

void MainWindow::setBackgroundColorB(int color) {
  float BColor = color / 99.;
	//std::cout << "B: " << BColor << "\n";
  ui->widget->setBackgroundColorB(BColor);
}

void MainWindow::setBackgroundColorG(int color) {
  float GColor = color / 99.;
	//std::cout << "G: " << GColor << "\n";
  ui->widget->setBackgroundColorG(GColor);
}

/*
 * Rendering Options Setting Functions
 * */

void MainWindow::setShowVertices(bool show) {
  ui->widget->setShowVertices(show);
}

void MainWindow::setShowTriangles(bool show) {
  ui->widget->setShowTriangles(show);
}

void MainWindow::setShowWire(bool show) { ui->widget->setShowWire(show); }

void MainWindow::setShowNormals(bool show) { ui->widget->setShowNormals(show); }

void MainWindow::setShowGrid(bool show) { ui->widget->setShowGrid(show); }

void MainWindow::setShowSolid(bool show) { ui->widget->setShowSolid(show); }

void MainWindow::setShowAxis(bool show) { ui->widget->setShowAxis(show); }

void MainWindow::setShowDatabase(bool show) { ui->dbwidget->setShowDatabase(show); }

void MainWindow::setShowDBVertices(bool show) {
  ui->dbwidget->setShowVertices(show);
}

void MainWindow::setShowDBTriangles(bool show) {
  ui->dbwidget->setShowTriangles(show);
}

void MainWindow::setShowDBFiltered(bool show) {
  ui->dbwidget->setShowFiltered(show);
}

void MainWindow::setShowTargetMesh(bool show) {
  ui->widget->setShowTargetMesh(show);
}

void MainWindow::setShowFilteredMesh(bool show) {
  ui->widget->setShowFilteredMesh(show);
}

void MainWindow::setMultiMesh(bool show) {
	ui->widget->setMultiMesh(show);
}

void MainWindow::setModelLighting(bool light) {
  ui->widget->setModelLighting(light);
}

void MainWindow::setNormalsLighting(bool light) {
  ui->widget->setNormalsLighting(light);
}

/*
 * Button Functions
 * */

void MainWindow::gridFilter() {
  //ui->widget->filtered_mesh = ui->widget->primary_meshes[0].partialView(0.33);
  //ui->widget->filtered_mesh.triangulate();
  ui->widget->filtered_mesh = ui->widget->primary_meshes[0].gridFilter();
	ui->widget->updateGL();
}

void MainWindow::onSegmentImg() {
	std::cout << "\n\n\e[1mDepth Image Segmentation begins..\e[0m\n";
  ImgSegmenter segm(ui->SnapShot_Widget->getImg(), median_kernel, normals_radius, edge_radius, kernel_radius);
  segm.estimateNormals();
  segm.printNormals();
  segm.detectNormalEdges();
  segm.colorRegions();

  ui->Normals_Widget->setImg(segm.norm_img);
  ui->Colored_Widget->setImg(segm.colored_img);
  ui->Binarized_Widget->setImg(segm.norm_bin_edge_img);
  segm.writetoMesh(ui->widget->primary_meshes, 1);
	//ui->widget->primary_meshes[0].triangulate();
  //segm.writetoMesh(ui->widget->primary_mesh, 1);

  //ui->widget->primary_mesh.computeNormals();
  //ui->widget->primary_mesh.preprocess();
	std::cout << "\e[1mDepth Image Segmentation ends..\e[0m\n";
}

void MainWindow::onGenerateMesh() {

//  cv::Mat depthMat(ui->SnapShot_Widget->getImg());
//  if (depthMat.empty()) {
//      std::cout << "There is no depth image! \n";
//      return;
//  }

  if (!ui->widget->primary_meshes[0].empty()) {
    ui->widget->primary_meshes[0].clear();
  }
  if (ui->Colored_Widget->mask_bool) {
    std::cout << "Writing masked mesh!\n";
		ImgSegmenter segm(ui->SnapShot_Widget->getImg());
		segm.estimateNormals();
    segm.writetoMesh(ui->widget->primary_meshes[0], 1, ui->Colored_Widget->getImg(), ui->Colored_Widget->mask);
    ui->Colored_Widget->mask_bool = false;
		ui->widget->primary_meshes[0].preprocess();
		ui->ChoosePointSpinBox->setMaximum(ui->widget->primary_meshes[0].vertices.size());
		ui->widget->updateGL();
  } else {
    //segm.writetoMesh(ui->widget->primary_mesh, 1);
		ui->widget->primary_mesh = ui->Kinect_Widget->generateMesh();
		ui->widget->primary_mesh.preprocess();
		ui->widget->primary_mesh.printInfo();
		ui->widget->updateGL();
  }
}

void MainWindow::onDraw() {
  ui->Colored_Widget->draw = !ui->Colored_Widget->draw;
  ui->Binarized_Widget->draw = !ui->Binarized_Widget->draw;
  ui->SnapShot_Widget->draw = !ui->SnapShot_Widget->draw;
  ui->Normals_Widget->draw = !ui->Normals_Widget->draw;
}

void MainWindow::onClearAll() {
  ui->widget->primary_mesh.clear();
	ui->widget->primary_meshes.clear();
	ui->widget->filtered_mesh.clear();
	ui->widget->vertex_chosen = false;
  ui->widget->updateGL();
  ui->Colored_Widget->clearImg();
  ui->Colored_Widget->updateGL();
  ui->Binarized_Widget->clearImg();
  ui->Binarized_Widget->updateGL();
  ui->Normals_Widget->clearImg();
  ui->Normals_Widget->updateGL();
}

void MainWindow::onBilateralFilter() {
	if (ui->widget->filtered_mesh.empty())
		ui->widget->filtered_mesh = ui->widget->primary_meshes[0].bilateralFilter(sd_bf, sn_bf);
	else 
		ui->widget->filtered_mesh = ui->widget->filtered_mesh.bilateralFilter(sd_bf, sn_bf);
	ui->widget->updateGL();
}

void MainWindow::onMultilateralFilter() {
	if (ui->widget->filtered_mesh.empty())
		ui->widget->filtered_mesh = ui->widget->primary_meshes[0].multilateralFilter(0.03);
	else 
		ui->widget->filtered_mesh = ui->widget->filtered_mesh.multilateralFilter(0.03);
	ui->widget->updateGL();
	ui->widget->filtered_mesh.computeNormals(1);
}

void MainWindow::onZoomImgWidget() {
  if (!zoom) {
    zoom = true;
    ui->Colored_Widget->setMinimumSize(ui->Colored_Widget->getImg().rows,
                                       ui->Kinect_Widget->width());
    ui->Binarized_Widget->setMinimumSize(0, 0);
    ui->SnapShot_Widget->setMinimumSize(0, 0);
    ui->Normals_Widget->setMinimumSize(0, 0);
  } else {
    zoom = false;
    ui->Colored_Widget->setMinimumSize(200, 150);
    ui->Binarized_Widget->setMinimumSize(200, 150);
    ui->SnapShot_Widget->setMinimumSize(200, 150);
    ui->Normals_Widget->setMinimumSize(200, 150);
  }

}

/*
 * Seemgentation Parameters
 * */

void MainWindow::setMedianKernel(int x) {
    median_kernel = x;
}

void MainWindow::setNormalsRadius(int x) {
    normals_radius = x;
}

void MainWindow::setEdgeRadius(int x) {
    edge_radius = x;
}

void MainWindow::setKernelRadius(int x) {
    kernel_radius = x;
}

void MainWindow::setGridSizeDB(double grid_size) {
	for (auto &m : ui->dbwidget->database_meshes)
		m.setGridSize(grid_size);
}

void MainWindow::setSdBF(double x) {
	sd_bf = x;
}

void MainWindow::setSnBF(double x) {
	sn_bf = x;
}
