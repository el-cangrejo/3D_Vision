#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "glwidget.h"
#include <QFileDialog>

#include <boost/filesystem.hpp>

extern Mesh filtered_mesh;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->ShowTrianglesCheckBox->setChecked(ui->widget->_showTriangles);
    ui->ShowSolidCheckBox->setChecked(ui->widget->_showSolid);
    ui->ShowAxisCheckBox->setChecked(ui->widget->_showAxis);
    ui->ModelColorBSlider->setSliderPosition((int)std::floor(ui->widget->_ModelColorB * 99.));
    ui->ModelColorRSlider->setSliderPosition((int)std::floor(ui->widget->_ModelColorR * 99.));
    ui->ModelColorGSlider->setSliderPosition((int)std::floor(ui->widget->_ModelColorG * 99.));
    ui->BackgroundColorRSlider->setSliderPosition(ui->widget->_BackgroundColorR * 99.);
    ui->BackgroundColorGSlider->setSliderPosition(ui->widget->_BackgroundColorB * 99.);
    ui->BackgroundColorBSlider->setSliderPosition(ui->widget->_BackgroundColorG * 99.);
    ui->RotFactorSlider->setSliderPosition(ui->widget->_rotFactor * 999.);
    ui->ZoomFactSlider->setSliderPosition(ui->widget->_zoomStep * 99.);

    connect(ui->ModelColorRSlider, SIGNAL(valueChanged (int)), this, SLOT(setModelColorR (int)));
    connect(ui->ModelColorGSlider, SIGNAL(valueChanged (int)), this, SLOT(setModelColorG (int)));
    connect(ui->ModelColorBSlider, SIGNAL(valueChanged (int)), this, SLOT(setModelColorB (int)));

    connect(ui->BackgroundColorRSlider, SIGNAL(valueChanged (int)), this, SLOT(setBackgroundColorR (int)));
    connect(ui->BackgroundColorGSlider, SIGNAL(valueChanged (int)), this, SLOT(setBackgroundColorG (int)));
    connect(ui->BackgroundColorBSlider, SIGNAL(valueChanged (int)), this, SLOT(setBackgroundColorB (int)));

    connect(ui->ShowVerticesCheckBox, SIGNAL(clicked (bool)), this, SLOT(setShowVertices (bool)));
    connect(ui->ShowTrianglesCheckBox, SIGNAL(clicked (bool)), this, SLOT(setShowTriangles (bool)));
    connect(ui->ShowWireCheckBox, SIGNAL(clicked (bool)), this, SLOT(setShowWire (bool)));
    connect(ui->ShowAxisCheckBox, SIGNAL(clicked (bool)), this, SLOT(setShowAxis (bool)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_action_Quit_triggered() {
    std::cout << "User evoked exiting..\nGoodbye!\n";
    exit(0);
}

void MainWindow::on_action_Open_triggered() {
    QString file_name = QFileDialog::getOpenFileName(this);
    if (!file_name.isEmpty()) {
        if (!ui->widget->primary_mesh.vertices.empty()) {
            Mesh new_mesh;
            read_mesh(file_name.toStdString(), new_mesh);
            ui->widget->primary_mesh = new_mesh;
        } else {
            read_mesh(file_name.toStdString(), ui->widget->primary_mesh);
        }
        ui->widget->primary_mesh.movetoCenter();
        ui->widget->primary_mesh.fittoUnitSphere();
        ui->widget->primary_mesh.computeNormals();
    }
    ui->widget->updateGL();
}

void MainWindow::on_GridFilter_clicked() {
    ui->widget->filtered_mesh = ui->widget->primary_mesh.gridFilter();
    ui->widget->updateGL();
}

void MainWindow::on_StatOutFIlter_clicked() {
    ui->widget->primary_mesh = ui->widget->primary_mesh.statoutFilter();
    preprocess_mesh(ui->widget->primary_mesh);
    ui->widget->updateGL();
}

void MainWindow::on_GridSize_valueChanged(double arg1) {
    ui->widget->primary_mesh.grid_size = arg1;
    std::cout << "Mesh's grid size set to " << arg1 << "\n";
}

void MainWindow::on_ShowFilteredMesh_clicked() {
    ui->widget->_showFilteredMesh = !ui->widget->_showFilteredMesh;
    ui->widget->updateGL();
}

void MainWindow::on_ShowGridFilter_clicked() {
    ui->widget->_showGrid =! ui->widget->_showGrid;
    ui->widget->updateGL();
}

void MainWindow::on_ShowNormalsCheckBox_clicked() {
    ui->widget->_showNormals = !ui->widget->_showNormals;
    ui->widget->updateGL();
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

void MainWindow::on_ShowSolidCheckBox_clicked() {
    ui->widget->_showSolid = !ui->widget->_showSolid;
    ui->widget->updateGL();
}

void MainWindow::on_action_Preprocess_Database_triggered() {
    QString directory_name = QFileDialog::getExistingDirectory(this);
    if (!directory_name.isEmpty()) {
        std::cout << "Given database to preprocess "
                  << " : " << directory_name.toStdString() << "\n";
        preprocess_database(directory_name.toStdString());
    }
}

void MainWindow::on_action_Load_Database_triggered() {
    QString directory_name = QFileDialog::getExistingDirectory(this);
    if (!directory_name.isEmpty()) {
        std::cout << "Given database to preprocess "
                  << " : " << directory_name.toStdString() << "\n";
        load_database(directory_name.toStdString(), ui->widget->db_descriptors, ui->widget->db_files);
        ui->widget->database = directory_name.toStdString();
    }
}

void MainWindow::on_SearchinDBButton_clicked() {
    if (ui->widget->db_descriptors.empty()) {
        std::cout << "No ldatabase loaded!\n";
        return;
    }

    float dist(10000000.0);
    int idx(0);

    std::cout << "Query mesh fpfh size : " << ui->widget->filtered_mesh.fpfhist.size() << "\n";
    std::cout << "Descriptors size : " << ui->widget->db_descriptors.size() << "\n";

    ui->widget->filtered_mesh.computeFPFH();

    for (size_t i = 0; i < ui->widget->db_descriptors.size(); ++i) {
        float new_dist = local_distance(ui->widget->filtered_mesh, ui->widget->db_descriptors[i]);
        if (new_dist < dist) {
            dist = new_dist;
            idx = i;
        }
    }
    std::cout << "Target mesh : " << idx << "\n";

    read_mesh(ui->widget->db_files[idx - 1], ui->widget->target_mesh);
    ui->widget->target_mesh.movetoCenter();
    ui->widget->target_mesh.fittoUnitSphere();
    ui->widget->_showTargetMesh = true;
    ui->widget->_showFilteredMesh = false;
    ui->widget->updateGL();
}

void MainWindow::on_EnableNormalsLighting_clicked()
{
    ui->widget->_normalLighting = !ui->widget->_normalLighting;
    ui->widget->updateGL();
}

void MainWindow::on_EnableLighting_clicked() {
    ui->widget->_enableLighting = !ui->widget->_enableLighting;
    ui->widget->updateGL();
}

/*
 * Color Setting Functions
 * */

void MainWindow::setModelColorR (int color) {
    float RColor = color / 99.;
    ui->widget->setModelColorR(RColor);
}

void MainWindow::setModelColorG (int color) {
    float GColor = color / 99.;
    ui->widget->setModelColorG(GColor);
}

void MainWindow::setModelColorB (int color) {
    float BColor = color / 99.;
    ui->widget->setModelColorB(BColor);
}

void MainWindow::setBackgroundColorR (int color) {
    float RColor = color / 99.;
    ui->widget->setBackgroundColorR(RColor);
}

void MainWindow::setBackgroundColorB (int color) {
    float BColor = color / 99.;
    ui->widget->setBackgroundColorB(BColor);
}

void MainWindow::setBackgroundColorG (int color) {
    float GColor = color / 99.;
    ui->widget->setBackgroundColorG(GColor);
}

/*
 * Rendering Options Setting Functions
 * */

void MainWindow::setShowVertices (bool show) {
    ui->widget->setShowVertices (show);
}

void MainWindow::setShowTriangles (bool show) {
    ui->widget->setShowTriangles (show);
}

void MainWindow::setShowWire (bool show) {
    ui->widget->setShowWire (show);
}

void MainWindow::setShowSolid (bool show) {
    ui->widget->setShowSolid (show);
}

void MainWindow::setShowAxis (bool show) {
    ui->widget->setShowAxis (show);
}
