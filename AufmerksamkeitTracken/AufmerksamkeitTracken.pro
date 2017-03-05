#-------------------------------------------------
#
# Project created by QtCreator 2016-11-09T20:36:58
#
# Installiere libvtk5-dev
# OpenCV: cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_VTK=ON -D BUILD_SHARED_LIBS=ON ..
# OpenFace: setze auf SHARED
# In Build-Ordner: mode/ CLNF kopieren
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AufmerksamkeitTracken
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    model/camera.cpp \
    control/facedetection.cpp \
    model/image.cpp \
    src/stdafx.cpp \
    model/imagesection.cpp \
    control/atentiontracer.cpp \
    control/frameevents.cpp \
    model/frame.cpp \
    model/box.cpp
#    FaceLandmarkVidMulti.cpp
#    FaceLandmarkImg.cpp

HEADERS  += mainwindow.h \
    model/camera.h \
    control/facedetection.h \
    model/image.h \
    src/algo.h \
    src/blob_gen.h \
    src/stdafx.h \
    model/imagesection.h \
    control/atentiontracer.h \
    control/frameevents.h \
    model/frame.h \
    model/box.h

FORMS    += mainwindow.ui

# Neu

PKGCONFIG += x11

PKGCONFIG += opencv
CONFIG += link_pkgconfig

#opencv
INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_viz -lopencv_core

#Boost:
INCLUDEPATH += /usr/include/boost
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_filesystem -lboost_system -lIlmImf -lz -lHalf -ljasper -ltiff -lpng12

#dlib
INCLUDEPATH += /home/falko/OpenFace/lib/3rdParty/dlib/include
LIBS += -L/home/falko/OpenFace/build/lib/3rdParty/dlib -ldlib

#tbb
LIBS += -ltbb -ltbbmalloc -ltbbmalloc_proxy

#blas and lapack
LIBS += -llapack -lblas

#openface
INCLUDEPATH += /home/falko/OpenFace/lib/local/LandmarkDetector/include
INCLUDEPATH += /home/falko/OpenFace/lib/local/FaceAnalyser/include
LIBS += -L/home/falko/OpenFace/build/lib/local/LandmarkDetector -lLandmarkDetector
LIBS += -L/home/falko/OpenFace/build/lib/local/FaceAnalyser -lFaceAnalyser
#LIBS += -L/home/pp/Librerias/opencv-3.1.0/build/3rdparty/lib -llibwebp
LIBS += -L/home/falko/OpenCV/3rdparty/ippicv/unpack/ippicv_lnx/lib/intel64 -lippicv

LIBS += -pthread -ljpeg
LIBS += -ldl -fPIC
