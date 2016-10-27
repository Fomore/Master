#-------------------------------------------------
#
# Project created by QtCreator 2016-10-27T13:36:39
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EllipseRechner
TEMPLATE = app

LIBS += `pkg-config opencv --libs`

SOURCES += main.cpp\
        mainwindow.cpp \
    Model/kreis.cpp

HEADERS  += mainwindow.h \
    Model/kreis.h

FORMS    += mainwindow.ui
