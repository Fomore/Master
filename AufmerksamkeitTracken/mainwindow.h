#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "control/facedetection.h"

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

private:
    Ui::MainWindow *ui;
    FaceDetection mFaceDetection;
};

#endif // MAINWINDOW_H
