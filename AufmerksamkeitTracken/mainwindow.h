#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"

#include "control/facedetection.h"
#include "control/frameevents.h"
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

private:
    Ui::MainWindow *ui;
//    Camera mKamera;
//    Image mImage;
    FaceDetection *mFaceDetection;
    FrameEvents *mFrameEvents;
    Camera* mKamera;
};

#endif // MAINWINDOW_H
