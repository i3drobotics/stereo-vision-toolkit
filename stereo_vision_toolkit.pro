#-------------------------------------------------
#
# Project created by QtCreator 2016-10-26T16:38:42
#
# Copyright I3D Robotics Ltd, 2020
# Authors: Josh Veitch-Michaelis, Ben Knight
#
#-------------------------------------------------

VERSION = 1.1.1

QT += core gui concurrent widgets xml network

TARGET = stereo_vision_toolkit
TEMPLATE = app vcapp

#CONFIG += console
CONFIG += warn_on
CONFIG += doc

#Comment out if not using I3DR's pro matcher
CONFIG += include_pro

include_pro {
    message("Pro enabled")
    DEFINES += BUILD_PRO
}

RC_FILE = icon.rc

RESOURCES += $$_PRO_FILE_PWD_/resources/qdarkstyle/style.qrc

include($$_PRO_FILE_PWD_/resources/QtAwesome/QtAwesome.pri)

VPATH = $$_PRO_FILE_PWD_/src
INCLUDEPATH += $$_PRO_FILE_PWD_/src

include_pro {
    INCLUDEPATH += $$_PRO_FILE_PWD_/pro/src
}

SOURCES += main.cpp\
        mainwindow.cpp \
    calibrationdialog.cpp \
    stereocalibrate.cpp \
    chessboard.cpp \
    calibrateconfirmdialog.cpp \
    calibratefromimagesdialog.cpp \
    abstractstereocamera.cpp \
    abstractstereomatcher.cpp \
    camerabasler.cpp \
    cameraopencv.cpp \
    stereocamerabasler.cpp \
    stereocameraopencv.cpp \
    stereocameradeimos.cpp \
    stereocameratis.cpp \
    stereocamerafromvideo.cpp \
    matcheropencvblock.cpp \
    matcherwidgetopencvblock.cpp \
    matcheropencvsgbm.cpp \
    matcherwidget.cpp \
    matcherwidgetopencvsgbm.cpp \
    disparityviewer.cpp \
    paramfile.cpp \
    cameradisplaywidget.cpp \
    cameraimagingsource.cpp

include_pro {
    SOURCES += \
        $$_PRO_FILE_PWD_/pro/src/matcherwidgetjrsgm.cpp \
        $$_PRO_FILE_PWD_/pro/src/matcherjrsgm.cpp
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm2.cpp
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm3.cpp
}

HEADERS  += mainwindow.h \
    calibrationdialog.h \
    stereocalibrate.h \
    chessboard.h \
    calibrateconfirmdialog.h \
    calibratefromimagesdialog.h \
    abstractstereocamera.h \
    abstractstereomatcher.h \
    camerabasler.h \
    cameraopencv.h \
    stereocamerabasler.h \
    stereocameraopencv.h \
    stereocameradeimos.h \
    stereocameratis.h \
    stereocamerafromvideo.h \
    matcheropencvblock.h \
    matcherwidgetopencvblock.h \
    matcheropencvsgbm.h \
    matcherwidget.h \
    matcherwidgetopencvsgbm.h \
    disparityviewer.h \
    paramfile.h \
    cameradisplaywidget.h \
    cameraimagingsource.h

include_pro {
    HEADERS += \
        $$_PRO_FILE_PWD_/pro/src/matcherwidgetjrsgm.h \
        $$_PRO_FILE_PWD_/pro/src/matcherjrsgm.h
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm2.h
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm3.h
}

FORMS    += mainwindow.ui \
    calibrationdialog.ui \
    calibrateconfirmdialog.ui \
    calibratefromimagesdialog.ui \
    matcherwidgetopencvblock.ui \
    matcherwidgetopencvsgbm.ui \
    disparityviewer.ui \
    cameradisplaywidget.ui

include_pro {
    FORMS += $$_PRO_FILE_PWD_/pro/src/matcherwidgetjrsgm.ui
}

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
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/tis/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/yaml-cpp/include"

CONFIG(debug, debug|release) {
    message("Debug mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/opencv/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/hidapi/lib/debug" -lhidapi
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/tis/lib/debug" -lTIS_UDSHL11d_x64
    LIBS += -lpcl_visualization_debug -lpcl_io_debug -lpcl_common_debug -lpcl_filters_debug
    LIBS += -lopencv_ximgproc341d -lopencv_core341d -lopencv_highgui341d -lopencv_calib3d341d -lopencv_videoio341d -lopencv_imgproc341d -lopencv_imgcodecs341d
    @#ifdef CUDA
    LIBS += -lopencv_cudastereo341d -lopencv_cudawarping341d
    @#endif
}else {
    message("Release mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/opencv/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/hidapi/lib/release" -lhidapi
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/tis/lib/release" -lTIS_UDSHL11_x64
    LIBS += -lpcl_visualization_release -lpcl_io_release -lpcl_common_release -lpcl_filters_release
    LIBS += -lopencv_ximgproc341 -lopencv_core341 -lopencv_highgui341 -lopencv_calib3d341 -lopencv_videoio341 -lopencv_imgproc341 -lopencv_imgcodecs341
    @#ifdef CUDA
    LIBS += -lopencv_cudastereo341 -lopencv_cudawarping341
    @#endif
}

LIBS += -lvtkCommonCore-7.0 -lvtkCommonDataModel-7.0 -lvtkGUISupportQt-7.0 -lvtkViewsQt-7.0 -lvtkViewsCore-7.0 -lvtkRenderingQt-7.0  -lvtkCommonMath-7.0 -lvtkRenderingCore-7.0 -lvtkIOCore-7.0

include_pro {
    # Required for JR
    LIBS += -L"$$_PRO_FILE_PWD_/pro/3rd_party/jr/lib" -lDigVTKIntegration
    INCLUDEPATH += "$$_PRO_FILE_PWD_/pro/3rd_party/jr/include"
    DEPENDPATH += "$$_PRO_FILE_PWD_/pro/3rd_party/jr/dep"

    # Required for JR2
    #LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/jr2/lib/PhobosIntegration" -lPhobosIntegration
    #INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/jr2/include"
    #DEPENDPATH += "$$_PRO_FILE_PWD_/3rd_party/jr2/dep"
}

# Required for PCL
LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/boost/lib"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/eigen"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/boost/include"

# Required for Basler
LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pylon/lib/x64"
#LIBS += -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lPylonBase_v6_0 -lPylonC -lPylonGUI_v6_0 -lPylonUtility_v6_0
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/pylon/include"

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

win32 {
    # define dlls to copy to build folder
    #$$files($$_PRO_FILE_PWD_/3rd_party/qt/*.dll, true) \
    EXTRA_BINFILES += \
        $$files($$_PRO_FILE_PWD_/3rd_party/opengl/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/opencv/dep/310/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/qt/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/cuda/bin/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/pylon/bin/*.dll, true)

    include_pro {
        EXTRA_BINFILES += \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/bin/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/dep/*.DLL, true) \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/lic/*.lic, true)
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/bin/*.dll, true) \
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/dep/*.dll, true) \
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/lic/*.lic, true)
    }

}

CONFIG( debug, debug|release ) {
    # debug
    DEPLOY_FOLDER = $$OUT_PWD/debug

    # define dlls to copy to build folder
    win32 {
        EXTRA_BINFILES += \
            $$_PRO_FILE_PWD_/3rd_party/hidapi/bin/Debug/hidapi.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/opencv/bin/debug/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pcl/bin/debug/*.dll, true) \
            $$_PRO_FILE_PWD_/3rd_party/png/libpng16d.dll \
            $$_PRO_FILE_PWD_/3rd_party/tbb/tbb_debug.dll \
            $$_PRO_FILE_PWD_/3rd_party/tis/bin/TIS_UDSHL11d_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/vtk/bin/debug/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/zlib/bin/debug/*.dll, true)
    }
} else {
    # release
    DEPLOY_FOLDER = $$OUT_PWD/release

    # define dlls to copy to build folder
    win32 {
        EXTRA_BINFILES += \
            $$_PRO_FILE_PWD_/3rd_party/hidapi/bin/Release/hidapi.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/opencv/bin/release/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pcl/bin/release/*.dll, true) \
            $$_PRO_FILE_PWD_/3rd_party/png/libpng16.dll \
            $$_PRO_FILE_PWD_/3rd_party/tbb/tbb.dll \
            $$_PRO_FILE_PWD_/3rd_party/tis/bin/TIS_UDSHL11_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/vtk/bin/release/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/zlib/bin/release/*.dll, true)
    }
}

# Deploy qt
## Uncomment the following line to help debug the deploy command when running qmake
#warning($${DEPLOY_COMMAND} $${DEPLOY_TARGET})
DEPLOY_TARGET = $$shell_quote($$shell_path($${DEPLOY_FOLDER}/$${TARGET}$${TARGET_CUSTOM_EXT}))
QMAKE_POST_LINK += $${DEPLOY_COMMAND} $${DEPLOY_TARGET}

win32 {
    #copy 3rd party dlls to build folder
    CONFIG += file_copies
    COPIES += extraDlls
    extraDlls.files = $${EXTRA_BINFILES}
    extraDlls.path = $${DEPLOY_FOLDER}
}

#copy documentation to build folder
COPIES += helpDocs
helpDocs.files = $$files($$_PRO_FILE_PWD_/docs/help/*.html, true)
helpDocs.files += $$files($$_PRO_FILE_PWD_/docs/help/*.png, true)
helpDocs.path = $${DEPLOY_FOLDER}/docs/help

CONFIG( doc ){
    QMAKE_POST_LINK += && cd $${_PRO_FILE_PWD_} && doxygen
}
