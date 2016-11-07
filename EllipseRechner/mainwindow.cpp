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

    x = y = r = s = d = 0;
    plot();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(plot()));
    timer->start(100);//Zeit in msec

//    mKreis.build_Table();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::plot(){
    if(x != ui->horizontalSlider->sliderPosition()
            || y != ui->verticalSlider->sliderPosition()
            || r != ui->verticalSlider_2->sliderPosition()
            || d != ui->verticalSlider_3->sliderPosition()
            || s != ui->verticalSlider_4->sliderPosition()){
        x = ui->horizontalSlider->sliderPosition();
        y = ui->verticalSlider->sliderPosition();
        r = ui->verticalSlider_2->sliderPosition();
        d = ui->verticalSlider_3->sliderPosition();
        s = ui->verticalSlider_4->sliderPosition();

        cv::Mat inMat = mKreis.print(x,y,r, d, s);
        cv::RotatedRect ellipse(mEllipse.calculate_Ellipse(inMat));

        mWinkel.calculate(ellipse,x,y);
        double tx,ty;
        mWinkel.calculate3(ellipse,tx,ty);
//        mWinkel.calculate2(ellipse,x,y);

        cv::ellipse( inMat, ellipse, cv::Scalar(255,0,255,255), 1,1 );
        cv::line(inMat,ellipse.center,ellipse.center+cv::Point2f(tx*30,ty*30),cv::Scalar(0,0,255,255), 1,1 );

        QImage image( inMat.data,
                      inMat.cols, inMat.rows,
                      static_cast<int>(inMat.step),
                      QImage::Format_ARGB32 );
        QPixmap img(QPixmap::fromImage(image));

        ui->label->setPixmap(img);
        ui->lineEdit->setText(QString::number(ui->horizontalSlider->sliderPosition()));
        ui->lineEdit_2->setText(QString::number(ui->verticalSlider->sliderPosition()));
        ui->lineEdit_3->setText(QString::number(ui->verticalSlider_2->sliderPosition()));
        ui->lineEdit_4->setText(QString::number(ui->verticalSlider_3->sliderPosition()));
        ui->lineEdit_5->setText(QString::number(ui->verticalSlider_4->sliderPosition()));
    }
}
