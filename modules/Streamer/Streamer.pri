QT += core gui widgets

DEPENDPATH += "$$PWD"
INCLUDEPATH += "$$PWD"
VPATH += "$$PWD"

SOURCES += \
        streamer.cpp \
        image2string.cpp \
        streamerviewer.cpp

HEADERS += \
        streamer.h \
        image2string.h \
        streamerviewer.h

FORMS += streamerviewer.ui
