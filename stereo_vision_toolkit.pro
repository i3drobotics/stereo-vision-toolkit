#-------------------------------------------------
#
# Project created by QtCreator 2016-10-26T16:38:42
#
# Copyright I3D Robotics Ltd, 2017
# Author: Josh Veitch-Michaelis
#
#-------------------------------------------------

QT       += core gui concurrent widgets xml

TARGET = stereo_vision_toolkit
TEMPLATE = app vcapp

#CONFIG += console
CONFIG += warn_on
CONFIG += doc

RC_FILE = icon.rc

RESOURCES += $$_PRO_FILE_PWD_/resources/qdarkstyle/style.qrc

include($$_PRO_FILE_PWD_/resources/QtAwesome/QtAwesome.pri)

VPATH = $$_PRO_FILE_PWD_/src
INCLUDEPATH += $$_PRO_FILE_PWD_/src

SOURCES += main.cpp\
        mainwindow.cpp \
    calibrationdialog.cpp \
    stereocalibrate.cpp \
    chessboard.cpp \
    calibrateconfirmdialog.cpp \
    calibratefromimagesdialog.cpp \
    abstractstereocamera.cpp \
    abstractstereomatcher.cpp \
    cameraopencv.cpp \
    stereocameraopencv.cpp \
    stereocameradeimos.cpp \
    matcheropencvblock.cpp \
    matcherwidgetopencvblock.cpp \
    stereocamerafromvideo.cpp \
    matcheropencvsgbm.cpp \
    matcherwidget.cpp \
    matcherwidgetopencvsgbm.cpp \
    disparityviewer.cpp \
    paramfile.cpp

HEADERS  += mainwindow.h \
    calibrationdialog.h \
    stereocalibrate.h \
    chessboard.h \
    calibrateconfirmdialog.h \
    calibratefromimagesdialog.h \
    abstractstereocamera.h \
    abstractstereomatcher.h \
    cameraopencv.h \
    stereocameraopencv.h \
    stereocameradeimos.h \
    matcheropencvblock.h \
    matcherwidgetopencvblock.h \
    stereocamerafromvideo.h \
    matcheropencvsgbm.h \
    matcherwidget.h \
    matcherwidgetopencvsgbm.h \
    disparityviewer.h \
    paramfile.h

FORMS    += mainwindow.ui \
    calibrationdialog.ui \
    calibrateconfirmdialog.ui \
    calibratefromimagesdialog.ui \
    matcherwidgetopencvblock.ui \
    matcherwidgetopencvsgbm.ui \
    disparityviewer.ui

# For building in a single folder
CONFIG(debug, debug|release) {
    DESTDIR = debug
    OBJECTS_DIR = .obj_debug
    MOC_DIR     = .moc_debug
}else {
    DESTDIR = release
    OBJECTS_DIR = .obj
    MOC_DIR     = .moc
}

INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/opencv/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/VTK/include/vtk-7.0"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/PCL/include/pcl-1.8"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/hidapi/include"

CONFIG(debug, debug|release) {
    message("Debug mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/opencv/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/hidapi/lib/debug" -lhidapi
    LIBS += -lpcl_visualization_debug -lpcl_io_debug -lpcl_common_debug -lpcl_filters_debug
    LIBS += -lopencv_ximgproc310d -lopencv_core310d -lopencv_highgui310d -lopencv_calib3d310d -lopencv_videoio310d -lopencv_imgproc310d -lopencv_imgcodecs310d -lopencv_cudastereo310d -lopencv_cudawarping310d
}else {
    message("Release mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/opencv/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/hidapi/lib/release" -lhidapi
    LIBS += -lpcl_visualization_release -lpcl_io_release -lpcl_common_release -lpcl_filters_release
    LIBS += -lopencv_ximgproc310 -lopencv_core310 -lopencv_highgui310 -lopencv_calib3d310 -lopencv_videoio310 -lopencv_imgproc310 -lopencv_imgcodecs310 -lopencv_cudastereo310 -lopencv_cudawarping310
}

LIBS += -lvtkCommonCore-7.0 -lvtkCommonDataModel-7.0 -lvtkGUISupportQt-7.0 -lvtkViewsQt-7.0 -lvtkViewsCore-7.0 -lvtkRenderingQt-7.0  -lvtkCommonMath-7.0 -lvtkRenderingCore-7.0 -lvtkIOCore-7.0

# Required for PCL
LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/boost/lib"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/eigen"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/boost/include"


# Directshow class IDs
LIBS += -lstrmiids

DISTFILES += $$_PRO_FILE_PWD_/resources/fonts/fontawesome-webfont.ttf

isEmpty(TARGET_EXT) {
    win32 {
        TARGET_CUSTOM_EXT = .exe
    }
    macx {
        TARGET_CUSTOM_EXT = .app
    }
} else {
    TARGET_CUSTOM_EXT = $${TARGET_EXT}
}

win32 {
    DEPLOY_COMMAND = windeployqt
}
macx {
    DEPLOY_COMMAND = macdeployqt
}

CONFIG( debug, debug|release ) {
    # debug
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/debug/$${TARGET}$${TARGET_CUSTOM_EXT}))
} else {
    # release
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/release/$${TARGET}$${TARGET_CUSTOM_EXT}))
}

#  # Uncomment the following line to help debug the deploy command when running qmake
#  warning($${DEPLOY_COMMAND} $${DEPLOY_TARGET})

# Use += instead of = if you use multiple QMAKE_POST_LINKs
QMAKE_POST_LINK += $${DEPLOY_COMMAND} $${DEPLOY_TARGET}

CONFIG( doc ){
    QMAKE_POST_LINK +=&& cd .. && doxygen
}
