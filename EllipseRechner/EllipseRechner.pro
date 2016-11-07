#-------------------------------------------------
#
# Project created by QtCreator 2016-10-27T13:36:39
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EllipseRechner
TEMPLATE = app

CONFIG += c++11

LIBS += `pkg-config opencv --libs`

SOURCES += main.cpp\
        mainwindow.cpp \
    Model/kreis.cpp \
    Model/ellipse.cpp \
    Control/neigungswinkel.cpp

HEADERS  += mainwindow.h \
    Model/kreis.h \
    Model/ellipse.h \
    Control/neigungswinkel.h

FORMS    += mainwindow.ui
