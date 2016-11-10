#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "model/camera.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Camera mKamera;
};

#endif // MAINWINDOW_H
