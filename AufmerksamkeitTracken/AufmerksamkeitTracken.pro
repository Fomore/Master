#-------------------------------------------------
#
# Project created by QtCreator 2016-11-09T20:36:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AufmerksamkeitTracken
TEMPLATE = app

CONFIG += c++11
CONFIG += link_pkgconfig

LIBS += `pkg-config opencv --libs`

SOURCES += main.cpp\
        mainwindow.cpp \
    model/camera.cpp \
    control/facedetection.cpp

HEADERS  += mainwindow.h \
    model/camera.h \
    control/facedetection.h

FORMS    += mainwindow.ui

# Neu

#Boost:
#INCLUDEPATH += /usr/include/boost
#LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_filesystem -lboost_system -lIlmImf -lz -lHalf -ljasper -ltiff -lpng12

#dlib
#INCLUDEPATH += /home/falko/Uni/Master/AufmerksamkeitTracken/lib/3rdParty/dlib/include
#LIBS += -L/home/falko/Uni/Master/AufmerksamkeitTracken/lib/3rdParty/dlib -ldlib

#tbb
#LIBS += -ltbb -ltbbmalloc -ltbbmalloc_proxy

#blas and lapack
#LIBS += -llapack -lblas

#openface
#INCLUDEPATH +=/home/falko/Uni/Master/AufmerksamkeitTracken/lib/local/LandmarkDetector/include ##correcto para include landmark.h
#LIBS += -L/home/falko/Uni/Master/AufmerksamkeitTracken/lib/local/LandmarkDetector -lLandmarkDetector
#LIBS += -L/home/pp/Librerias/opencv-3.1.0/build/3rdparty/lib -llibwebp
#LIBS += -L/home/pp/Librerias/opencv-3.1.0/3rdparty/ippicv/unpack/ippicv_lnx/lib/intel64 -lippicv

#LIBS += -pthread -ljpeg
#LIBS += -ldl -fPIC
