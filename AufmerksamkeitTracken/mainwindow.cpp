#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QInputDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mKamera = new Camera(8);

    mKamera->setPath("/home/falko/Uni/Master/Film/Schulklasse/23100601S1.avi");
    //mKamera->setPath("/home/falko/Uni/Master/Film/VideoMessung/falko_no_glasses.avi");
    //mKamera->setPath("/home/falko/Uni/Master/Film/Test_Positionen_1.mp4");

    /*
    for(int i = 1; i < 2 ; i++){
        mEventHandler->loadXML("/home/falko/Uni/Master/Film/Schulklasse/23100601S1/23100601S1_"+QString::number(i)+"_Label.xml", i == 1);
    }
    */
    mFaceDetection = new FaceDetection(ui,mKamera);
    mFaceDetection->loadXML("/home/falko/Uni/Hiwi/build-VideoLabel-Desktop-Debug/data/23100601S1_Label.xml", true);
    //mFaceDetection->loadXML("/home/falko/Uni/Hiwi/build-VideoLabel-Desktop-Debug/data/23100601S1_Gaze_3_Label.xml", true);
    //mFaceDetection->loadXML("/home/falko/Uni/Hiwi/build-VideoLabel-Desktop-Debug/data/Test_Positionen_1_Label.xml", true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    if(ui->actionVerbesserter_Ablauf->isChecked()){
        mFaceDetection->FaceTrackingNewVersion();
    }else{
        mFaceDetection->FaceTracking();
    }
}

void MainWindow::on_Learn_Button_clicked()
{
    if(ui->actionFrom_XML_File->isChecked()){
        mFaceDetection->ShowFromeFile();
    }else{
        mFaceDetection->FaceTrackingImage();
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
        size_t anzFace = mFaceDetection->loadXML(filename,true);
        mFaceDetection->setMaxFaces(anzFace);
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
    int i = QInputDialog::getInt(this,"Kamera","Wähle eine Kamera-Einstellung:\n1: Webcam\n2: 1280P der 4k Actioncam (als Webcam)\n3: 1940P der 4K Actioncam (1080P Einstellung)\n4: 2688P der 4k Actioncam (2.7K Einstellung)\n5: 2688P der 4k Actioncam (2.7K Einstellung in Box)\n7: 3840P der 4K Actioncam\n8: Logitech c920",
                                 mKamera->getCameraID(),0,10,1);
    mKamera->setCameraParameter(i);
}

void MainWindow::on_actionUse_Boxes_triggered(bool checked)
{
    mFaceDetection->setUseBox(checked);
    mFaceDetection->setUseTime(checked);
    ui->actionUse_Video->setChecked(!checked);
}

void MainWindow::on_actionUse_Video_triggered(bool checked)
{
    mFaceDetection->setUseTime(checked);
    mFaceDetection->setUseBox(!checked);
    ui->actionUse_Boxes->setChecked(!checked);
}

void MainWindow::on_actionCorrect_Image_triggered(bool checked)
{
    mKamera->setUseCorrection(checked);
}
void MainWindow::on_actionLearn_Model_triggered(bool checked)
{
    mFaceDetection->setLearn(checked);
}

void MainWindow::on_actionUse_CLAHE_triggered(bool checked)
{
    mFaceDetection->setCLAHE(checked);
}

void MainWindow::on_actionAugen_Mitteln_triggered(bool checked)
{
    mFaceDetection->setUseEye(checked);
}

void MainWindow::on_actionSet_fx_fy_triggered()
{
    double fx = QInputDialog::getDouble(this,"Brennweite","Eingabe der Brennweite fx",
                                  mKamera->getFx(),-1);
    double fy = QInputDialog::getDouble(this,"Brennweite","Eingabe der Brennweite fy",
                                  mKamera->getFy(),-1);
    mKamera->setFxFy(fx,fy);
}

void MainWindow::on_actionScale_Box_triggered()
{
    double s = QInputDialog::getDouble(this,"Skallierung","Um welchen Faktor soll die Box (XML) verändert werden?",
                                  1.0,0,300,2);
    mFaceDetection->setBoxScall(s);
}

void MainWindow::on_actionSet_min_Box_Size_triggered()
{
    int w = QInputDialog::getInt(this,"Box Width","Mindesbreite der Box",100,1);
    int h = QInputDialog::getInt(this,"Box Height","Mindeshöhe der Box",100,1);

    mFaceDetection->setBoxMinSize(w,h);
}

void MainWindow::on_actionShow_Eyes_triggered(bool checked)
{
    mFaceDetection->setShowEyes(checked);
}

void MainWindow::on_actionSave_Image_triggered(bool checked)
{
    mFaceDetection->setSaveIamge(checked);
}

void MainWindow::on_actionWrite_Solution_triggered(bool checked)
{
    mFaceDetection->setWriteSolution(checked);
}

void MainWindow::on_actionShow_Atention_triggered(bool checked)
{
    mFaceDetection->setShowAtention(checked);
}

void MainWindow::on_actionShow_Landmarks_triggered(bool checked)
{
    mFaceDetection->setShowLandmarks(checked);
}

void MainWindow::on_actionBeobachte_Gaze_triggered()
{
    mFaceDetection->setGaze(QInputDialog::getInt(this,"Select Gaze","GAZE_YET_UNCHANGED = 0\n UNKNOWN = 1\n BLACKBOARD = 2\n DESK = 3\n WINDOW = 4\n TEACHER = 5\n CONTRIBUTING_STUDENT = 6\n OTHER_STUDENT = 7",
                                                 mFaceDetection->getGaze(),0,7,1));
}

void MainWindow::on_actionShow_Image_Box_triggered(bool checked)
{
    mFaceDetection->setShowImageBox(checked);
}

void MainWindow::on_actionShow_Head_Box_triggered(bool checked)
{
    mFaceDetection->setShowHeadBox(checked);
}
