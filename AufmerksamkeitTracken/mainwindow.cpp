#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QInputDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mKamera = new Camera(5);
    mFrameEvents = new FrameEvents();

    mKamera->setPath("/home/falko/Uni/Master/Film/Test_Positionen_1.mp4");
    mFrameEvents->loadXML("/home/falko/Uni/Hiwi/build-VideoLabel-Desktop-Debug/data/Test_Positionen_1_Label.xml");

    mFaceDetection = new FaceDetection(ui,mFrameEvents,mKamera);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    if(!ui->checkBox_Autorun->isChecked()){
        mFaceDetection->FaceTracking();
    }else{
        mFaceDetection->FaceTrackingAutoSize();
    }
}

void MainWindow::on_Learn_Button_clicked()
{
    mFaceDetection->LearnModel();
}

void MainWindow::on_actionOpen_Video_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this,tr("Open Video"), "~", tr("Video Files (*.avi *.mp4 *.wmv);; All (*.*)"));
    if(!filename.isEmpty() && filename.size() > 0){
        if(mKamera->setPath(filename)){
            ui->statusBar->showMessage("Video geladen");
        }else{
            ui->statusBar->showMessage("Fehler: Video Datei nicht gefunden");
        }
    }
}

void MainWindow::on_actionOpen_XML_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this,tr("Open XML-Datei"), "~", tr("XML (*.xml);; All (*.*)"));
    if(!filename.isEmpty() && filename.size() > 0){
        mFrameEvents->loadXML(filename);
    }
}

void MainWindow::on_actionMax_Faces_triggered()
{
    int i = QInputDialog::getInt(this,"Max Faces","Anzahl der gleichzeitig erkannten Gesichter",
                                 mFaceDetection->getMaxFaces(),1,100,1);
    mFaceDetection->setMaxFaces(i);
}
