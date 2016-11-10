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

LIBS += `pkg-config opencv --libs`

SOURCES += main.cpp\
        mainwindow.cpp \
    model/camera.cpp

HEADERS  += mainwindow.h \
    model/camera.h

FORMS    += mainwindow.ui
