#-------------------------------------------------
#
# Project created by QtCreator 2018-01-01T07:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets

TARGET = ExperimentCOMPLEXITY
TEMPLATE = lib

DEFINES += EXPERIMENTCEEXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    complexityExperiment.cpp \
    complexityEnvironment.cpp

HEADERS +=\
    kilobot.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    global.h \
    resources.h \
    area.h \
    complexityExperiment.h \
    complexityEnvironment.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core
