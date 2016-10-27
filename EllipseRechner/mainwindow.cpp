#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <QTimer>
#include <QPainter>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    cv::Mat inMat = mKreis.print(0,0);
    QImage image( inMat.data,
                  inMat.cols, inMat.rows,
                  static_cast<int>(inMat.step),
                  QImage::Format_ARGB32 );
    ui->label->setPixmap(QPixmap::fromImage(image));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(plot()));
    timer->start(100);//Zeit in msec
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::plot(){
    cv::Mat inMat = mKreis.print(ui->horizontalSlider->sliderPosition(),ui->verticalSlider->sliderPosition());
    QImage image( inMat.data,
                  inMat.cols, inMat.rows,
                  static_cast<int>(inMat.step),
                  QImage::Format_ARGB32 );
    QPixmap img(QPixmap::fromImage(image));

    QPainter paint(&img);

    paint.translate(300,300);
    paint.rotate(90);
    paint.setPen(*(new QColor(255,34,255,255)));
    int x = 100;
    int y = 150;
    paint.drawEllipse(-x/2,-y/2,x,y);

    ui->label->setPixmap(img);
    ui->lineEdit->setText(QString::number(ui->horizontalSlider->sliderPosition()));
    ui->lineEdit_2->setText(QString::number(ui->verticalSlider->sliderPosition()));
}
