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
    mFrameEvents->loadXML("/home/falko/Uni/Hiwi/build-VideoLabel-Desktop-Debug/data/Test_Positionen_1_img.xml");

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
    if(ui->actionFrom_XML_File->isChecked()){
        mFaceDetection->ShowFromeFile();
    }else{
        mFaceDetection->LearnModel();
    }
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

void MainWindow::on_actionAuto_Size_triggered(bool checked)
{
    mFaceDetection->setAutoSize(checked);
}

void MainWindow::on_actionSelect_Camera_triggered()
{
    int i = QInputDialog::getInt(this,"Kamera","Wähle eine Kamera-Einstellung:\n1: Webcam\n2: 1280P der 4k Actioncam (als Webcam)\n3: 1940P der 4K Actioncam (1080P Einstellung)\n4: 2688P der 4k Actioncam (2.7K Einstellung)\n5: 2688P der 4k Actioncam (2.7K Einstellung in Box)\n7: 3840P der 4K Actioncam",
                                 mKamera->getCameraID(),0,10,1);
    mKamera->setCameraParameter(i);
}

void MainWindow::on_actionUse_Boxes_triggered(bool checked)
{
    mFaceDetection->setUseBox(checked);
}

void MainWindow::on_actionCorrect_Image_triggered(bool checked)
{
    mKamera->setUseCorrection(checked);
}
void MainWindow::on_actionLearn_Model_triggered(bool checked)
{
    mFaceDetection->setLearn(checked);
}
