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
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionShow_Vertices_triggered() {
    ui->widget->_showVerts = !ui->widget->_showVerts;
    ui->widget->updateGL();
}

void MainWindow::on_action_Show_Triangles_triggered() {
    ui->widget->_showTriangles = !ui->widget->_showTriangles;
    ui->widget->updateGL();
}

void MainWindow::on_actionShow_Normals_triggered() {
    ui->widget->_showNormals = !ui->widget->_showNormals;
    ui->widget->updateGL();
}

void MainWindow::on_action_Quit_triggered() {
    std::cout << "User evoked exiting..\nGoodbye!\n";
    exit(0);
}

void MainWindow::on_action_Show_Grid_triggered() {
    ui->widget->_showGrid = !ui->widget->_showGrid;
    ui->widget->updateGL();
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

void MainWindow::on_actionShow_Filtered_Mesh_triggered() {
    ui->widget->_showFilteredMesh = !ui->widget->_showFilteredMesh;
    ui->widget->updateGL();
}

void MainWindow::on_Rbackcolor_sliderMoved(int position) {
    ui->widget->_BackgroundColorR = (float) position / 99;
    ui->widget->updateGL();
}

void MainWindow::on_Bbackcolor_sliderMoved(int position) {
    ui->widget->_BackgroundColorG = (float) position / 99;
    ui->widget->updateGL();
}

void MainWindow::on_horizontalSlider_2_sliderMoved(int position) {
    ui->widget->_BackgroundColorB = (float) position / 99;
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

void MainWindow::on_ShowWireCheckBox_clicked() {
    ui->widget->_showWire =! ui->widget->_showWire;
    ui->widget->updateGL();
}

void MainWindow::on_ShowVerticesCheckBox_clicked() {
    ui->widget->_showVerts = !ui->widget->_showVerts;
    ui->widget->updateGL();
}

void MainWindow::on_ShowTrianglesCheckBox_clicked() {
    ui->widget->_showTriangles = !ui->widget->_showTriangles;
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

void MainWindow::on_ModelColorRSlider_sliderMoved(int position) {
    ui->widget->_ModelColorR = position / 99.;
    ui->widget->updateGL();
}

void MainWindow::on_ModelColorGSlider_sliderMoved(int position) {
    ui->widget->_ModelColorG = position / 99.;
    ui->widget->updateGL();
}

void MainWindow::on_ModelColorBSlider_sliderMoved(int position) {
    ui->widget->_ModelColorB = position / 99.;
    ui->widget->updateGL();
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

void MainWindow::on_ShowAxis_clicked() {
    ui->widget->_showAxis = !ui->widget->_showAxis;
    ui->widget->updateGL();
}

void MainWindow::on_EnableNormalsLighting_clicked()
{
    ui->widget->_normalLighting = !ui->widget->_normalLighting;
    ui->widget->updateGL();
}
