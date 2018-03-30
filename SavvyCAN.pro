#-------------------------------------------------
#
# Project created by QtCreator 2015-04-25T22:57:44
#
#-------------------------------------------------

QT = core gui printsupport qml serialbus serialport widgets

CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT

CONFIG += c++11

DEFINES += QCUSTOMPLOT_USE_OPENGL
DEFINES += VENDOR_SAPA

TARGET = SavvyCAN
TEMPLATE = app

SOURCES += main.cpp\
    mainwindow.cpp \
    canframemodel.cpp \
    utility.cpp \
    qcustomplot.cpp \
    frameplaybackwindow.cpp \
    candatagrid.cpp \
    framesenderwindow.cpp \
    framefileio.cpp \
    mainsettingsdialog.cpp \
    firmwareuploaderwindow.cpp \
    scriptingwindow.cpp \
    scriptcontainer.cpp \
    canfilter.cpp \
    can_structs.cpp \
    motorcontrollerconfigwindow.cpp \
    connections/canconnection.cpp \
    connections/serialbusconnection.cpp \
    connections/canconfactory.cpp \
    connections/gvretserial.cpp \
    connections/canconmanager.cpp \
    re/sniffer/snifferitem.cpp \
    re/sniffer/sniffermodel.cpp \
    re/sniffer/snifferwindow.cpp \
    dbc/dbc_classes.cpp \
    dbc/dbchandler.cpp \
    dbc/dbcloadsavewindow.cpp \
    dbc/dbcmaineditor.cpp \
    dbc/dbcsignaleditor.cpp \
    re/discretestatewindow.cpp \
    re/filecomparatorwindow.cpp \
    re/flowviewwindow.cpp \
    re/frameinfowindow.cpp \
    re/fuzzingwindow.cpp \
    re/isotp_interpreterwindow.cpp \
    re/rangestatewindow.cpp \
    re/udsscanwindow.cpp \
    connections/canbus.cpp \
    connections/canconnectionmodel.cpp \
    connections/connectionwindow.cpp \
    re/graphingwindow.cpp \
    re/newgraphdialog.cpp \
    bisectwindow.cpp \
    signalviewerwindow.cpp \
    bus_protocols/isotp_handler.cpp \
    bus_protocols/j1939_handler.cpp \
    bus_protocols/uds_handler.cpp \
    jsedit.cpp \
    frameplaybackobject.cpp

HEADERS  += mainwindow.h \
    can_structs.h \
    canframemodel.h \
    utility.h \
    qcustomplot.h \
    frameplaybackwindow.h \
    candatagrid.h \
    framesenderwindow.h \
    can_trigger_structs.h \
    framefileio.h \
    config.h \
    mainsettingsdialog.h \
    firmwareuploaderwindow.h \
    scriptingwindow.h \
    scriptcontainer.h \
    canfilter.h \
    utils/lfqueue.h \
    motorcontrollerconfigwindow.h \
    connections/canconnection.h \
    connections/serialbusconnection.h \
    connections/canconconst.h \
    connections/canconfactory.h \
    connections/gvretserial.h \
    connections/canconmanager.h \
    re/sniffer/snifferitem.h \
    re/sniffer/sniffermodel.h \
    re/sniffer/snifferwindow.h \
    dbc/dbc_classes.h \
    dbc/dbchandler.h \
    dbc/dbcloadsavewindow.h \
    dbc/dbcmaineditor.h \
    dbc/dbcsignaleditor.h \
    re/discretestatewindow.h \
    re/filecomparatorwindow.h \
    re/flowviewwindow.h \
    re/frameinfowindow.h \
    re/fuzzingwindow.h \
    re/isotp_interpreterwindow.h \
    re/rangestatewindow.h \
    re/udsscanwindow.h \
    connections/canbus.h \
    connections/canconnectionmodel.h \
    connections/connectionwindow.h \
    re/graphingwindow.h \
    re/newgraphdialog.h \
    bisectwindow.h \
    signalviewerwindow.h \
    bus_protocols/isotp_handler.h \
    bus_protocols/j1939_handler.h \
    bus_protocols/uds_handler.h \
    bus_protocols/isotp_message.h \
    jsedit.h \
    frameplaybackobject.h

FORMS    += ui/candatagrid.ui \
    ui/connectionwindow.ui \
    ui/dbcloadsavewindow.ui \
    ui/dbcmaineditor.ui \
    ui/dbcsignaleditor.ui \
    ui/discretestatewindow.ui \
    ui/filecomparatorwindow.ui \
    ui/firmwareuploaderwindow.ui \
    ui/flowviewwindow.ui \
    ui/frameinfowindow.ui \
    ui/frameplaybackwindow.ui \
    ui/framesenderwindow.ui \
    ui/fuzzingwindow.ui \
    ui/graphingwindow.ui \
    ui/isotp_interpreterwindow.ui \
    ui/mainsettingsdialog.ui \
    ui/mainwindow.ui \
    ui/motorcontrollerconfigwindow.ui \
    ui/newgraphdialog.ui \
    ui/rangestatewindow.ui \
    ui/scriptingwindow.ui \
    ui/snifferwindow.ui \
    ui/udsscanwindow.ui \
    ui/bisectwindow.ui \
    ui/signalviewerwindow.ui

DISTFILES +=

RESOURCES += \
    icons.qrc \
    images.qrc

win32 {
#   LIBS += opengl32.lib
}

if (contains(DEFINES, VENDOR_SAPA)) {
    TARGET = SCSapa

    SOURCES += connections/wizbuserial.cpp
    HEADERS += connections/wizbuserial.h
    HEADERS += jshelper.h
    SOURCES += ccrashstack.cpp
    HEADERS += ccrashstack.h

    LIBS -= opengl32.lib
}
