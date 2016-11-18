#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"

#include "control/facedetection.h"
#include "model/camera.h"

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
    void showImage(cv::Mat image);

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    Camera mKamera;
    FaceDetection *mFaceDetection;
};

#endif // MAINWINDOW_H
