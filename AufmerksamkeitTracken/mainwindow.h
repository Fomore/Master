#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"

#include "control/facedetection.h"
#include "model/camera.h"
#include "model/image.h"

#include <opencv2/opencv.hpp>

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
    void on_pushButton_clicked();

    void on_Learn_Button_clicked();

    void on_actionOpen_Video_triggered();

    void on_actionOpen_XML_triggered();

    void on_actionMax_Faces_triggered();

    void on_actionAuto_Size_triggered(bool checked);

    void on_actionSelect_Camera_triggered();

    void on_actionUse_Boxes_triggered(bool checked);

    void on_actionCorrect_Image_triggered(bool checked);

    void on_actionLearn_Model_triggered(bool checked);

    void on_actionUse_CLAHE_triggered(bool checked);

    void on_actionAugen_Mitteln_triggered(bool checked);

    void on_actionSet_fx_fy_triggered();

    void on_actionScale_Box_triggered();

    void on_actionSet_min_Box_Size_triggered();

    void on_actionShow_Eyes_triggered(bool checked);

    void on_actionSave_Image_triggered(bool checked);

    void on_actionWrite_Solution_triggered(bool checked);

    void on_actionShow_Atention_triggered(bool checked);

    void on_actionShow_Landmarks_triggered(bool checked);

    void on_actionBeobachte_Gaze_triggered();

private:
    Ui::MainWindow *ui;
//    Image mImage;
    FaceDetection *mFaceDetection;
    Camera* mKamera;
};

#endif // MAINWINDOW_H
