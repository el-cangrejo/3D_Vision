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
    void on_action_Quit_triggered();

    void on_action_Open_triggered();

    void on_GridFilter_clicked();

    void on_StatOutFIlter_clicked();

    void on_GridSize_valueChanged(double arg1);

    void on_ShowFilteredMesh_clicked();

    void on_ShowGridFilter_clicked();

    void on_ShowNormalsCheckBox_clicked();

    void on_NormalsLengthSlider_valueChanged(int value);

    void on_ZoomFactSlider_sliderMoved(int position);

    void on_RotFactorSlider_sliderMoved(int position);

    void on_ShowSolidCheckBox_clicked();

    void on_action_Preprocess_Database_triggered();

    void on_action_Load_Database_triggered();

    void on_SearchinDBButton_clicked();

    void on_EnableNormalsLighting_clicked();

    void on_EnableLighting_clicked();

    /*
     * Color setting functions
     * */

    void setModelColorR (int);
    void setModelColorG (int);
    void setModelColorB (int);

    void setBackgroundColorR (int);
    void setBackgroundColorB (int);
    void setBackgroundColorG (int);

    /*
     * Rendering Options Setting Functions
     * */

    void setShowVertices (bool);
    void setShowTriangles (bool);
    void setShowWire (bool);
    void setShowSolid (bool);
    void setShowAxis (bool);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
