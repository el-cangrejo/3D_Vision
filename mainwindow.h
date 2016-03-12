#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionShow_Vertices_triggered();

    void on_action_Quit_triggered();

    void on_action_Show_Triangles_triggered();

    void on_action_Open_triggered();

    void on_actionShow_Normals_triggered();

    void on_action_Show_Grid_triggered();

    void on_actionShow_Filtered_Mesh_triggered();

    void on_Rbackcolor_sliderMoved(int position);

    void on_Bbackcolor_sliderMoved(int position);

    void on_horizontalSlider_2_sliderMoved(int position);

    void on_GridFilter_clicked();

    void on_StatOutFIlter_clicked();

    void on_GridSize_valueChanged(double arg1);

    void on_ShowFilteredMesh_clicked();

    void on_ShowGridFilter_clicked();

    void on_ShowWireCheckBox_clicked();

    void on_ShowVerticesCheckBox_clicked();

    void on_ShowTrianglesCheckBox_clicked();

    void on_ShowNormalsCheckBox_clicked();

    void on_NormalsLengthSlider_valueChanged(int value);

    void on_ZoomFactSlider_sliderMoved(int position);

    void on_RotFactorSlider_sliderMoved(int position);

    void on_ModelColorRSlider_sliderMoved(int position);

    void on_ModelColorGSlider_sliderMoved(int position);

    void on_ModelColorBSlider_sliderMoved(int position);

    void on_ShowSolidCheckBox_clicked();

    void on_action_Preprocess_Database_triggered();

    void on_action_Load_Database_triggered();

    void on_SearchinDBButton_clicked();

    void on_ShowAxis_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
