#-------------------------------------------------
#
# Project created by QtCreator 2017-05-09T22:30:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Basisanwendung
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp

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
