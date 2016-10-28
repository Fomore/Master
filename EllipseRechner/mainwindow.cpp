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

    x = y = 180;
    plot();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(plot()));
    timer->start(100);//Zeit in msec
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::plot(){
    if(x != ui->horizontalSlider->sliderPosition() || y != ui->verticalSlider->sliderPosition()){
        x = ui->horizontalSlider->sliderPosition();
        y = ui->verticalSlider->sliderPosition();


        cv::Mat inMat = mKreis.print(x,y);
        cv::ellipse( inMat, mEllipse.calculate_Ellipse(inMat), cv::Scalar(255,0,255), 1,1 );

        mEllipse.calculate_Ellipse(inMat);

        QImage image( inMat.data,
                      inMat.cols, inMat.rows,
                      static_cast<int>(inMat.step),
                      QImage::Format_ARGB32 );
        QPixmap img(QPixmap::fromImage(image));

        ui->label->setPixmap(img);
        ui->lineEdit->setText(QString::number(ui->horizontalSlider->sliderPosition()));
        ui->lineEdit_2->setText(QString::number(ui->verticalSlider->sliderPosition()));
    }
}
