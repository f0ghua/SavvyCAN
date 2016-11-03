#-------------------------------------------------
#
# Project created by QtCreator 2015-04-25T22:57:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets serialport printsupport qml

DEFINES += VENDOR_SAPA
CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT

CONFIG += c++11 qscintilla2

TARGET = SavvyCAN
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    canframemodel.cpp \
    utility.cpp \
    qcustomplot.cpp \
    graphingwindow.cpp \
    frameinfowindow.cpp \
    newgraphdialog.cpp \
    frameplaybackwindow.cpp \
    serialworker.cpp \
    candatagrid.cpp \
    flowviewwindow.cpp \
    framesenderwindow.cpp \
    dbchandler.cpp \
    dbcmaineditor.cpp \
    dbcsignaleditor.cpp \
    framefileio.cpp \
    filecomparatorwindow.cpp \
    mainsettingsdialog.cpp \
    firmwareuploaderwindow.cpp \
    discretestatewindow.cpp \
    connectionwindow.cpp \
    scriptingwindow.cpp \
    scriptcontainer.cpp \
    canfilter.cpp \
    rangestatewindow.cpp \
    dbc_classes.cpp \
    dbcloadsavewindow.cpp \
    fuzzingwindow.cpp \
    udsscanwindow.cpp \
    can_structs.cpp \
    isotp_interpreterwindow.cpp \
    isotp_handler.cpp \
    motorcontrollerconfigwindow.cpp

HEADERS  += mainwindow.h \
    can_structs.h \
    canframemodel.h \
    utility.h \
    qcustomplot.h \
    graphingwindow.h \
    frameinfowindow.h \
    newgraphdialog.h \
    frameplaybackwindow.h \
    serialworker.h \
    candatagrid.h \
    flowviewwindow.h \
    framesenderwindow.h \
    can_trigger_structs.h \
    dbc_classes.h \
    dbchandler.h \
    dbcmaineditor.h \
    dbcsignaleditor.h \
    framefileio.h \
    config.h \
    filecomparatorwindow.h \
    mainsettingsdialog.h \
    firmwareuploaderwindow.h \
    discretestatewindow.h \
    connectionwindow.h \
    scriptingwindow.h \
    scriptcontainer.h \
    canfilter.h \
    rangestatewindow.h \
    dbcloadsavewindow.h \
    fuzzingwindow.h \
    udsscanwindow.h \
    isotp_interpreterwindow.h \
    isotp_handler.h \
    motorcontrollerconfigwindow.h

FORMS    += mainwindow.ui \
    graphingwindow.ui \
    frameinfowindow.ui \
    newgraphdialog.ui \
    frameplaybackwindow.ui \
    candatagrid.ui \
    flowviewwindow.ui \
    framesenderwindow.ui \
    dbcmaineditor.ui \
    dbcsignaleditor.ui \
    filecomparatorwindow.ui \
    mainsettingsdialog.ui \
    firmwareuploaderwindow.ui \
    discretestatewindow.ui \
    connectionwindow.ui \
    scriptingwindow.ui \
    rangestatewindow.ui \
    dbcloadsavewindow.ui \
    fuzzingwindow.ui \
    udsscanwindow.ui \
    isotp_interpreterwindow.ui \
    deltawindow.ui \
    motorcontrollerconfigwindow.ui

DISTFILES +=

RESOURCES += \
    icons.qrc \
    images.qrc
