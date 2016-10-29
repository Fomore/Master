#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "Model/kreis.h"
#include "Model/ellipse.h"

#include "Control/neigungswinkel.h"

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

    void plot();

private:
    Ui::MainWindow *ui;
    Kreis mKreis;
    Ellipse mEllipse;
    Neigungswinkel mWinkel;
    int x,y;
};

#endif // MAINWINDOW_H
