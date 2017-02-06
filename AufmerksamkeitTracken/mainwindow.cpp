#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mFaceDetection = new FaceDetection(ui);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    if(!ui->checkBox_Autorun->isChecked()){
        mFaceDetection->FaceTracking(ui->Path_lineEdit->text().toStdString());
    }else{
        mFaceDetection->FaceTrackingAutoSize(ui->Path_lineEdit->text().toStdString());
    }
}

void MainWindow::on_Learn_Button_clicked()
{
    mFaceDetection->LearnModel();
}
