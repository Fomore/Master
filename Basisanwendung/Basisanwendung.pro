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

LIBS += -L/home/falko/OpenCV/3rdparty/ippicv/unpack/ippicv_lnx/lib/intel64 -lippicv
