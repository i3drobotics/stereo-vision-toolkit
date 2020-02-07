#-------------------------------------------------
#
# Stereo Vision Toolkit
#
# Copyright I3D Robotics Ltd, 2020
# Authors: Josh Veitch-Michaelis, Ben Knight
#
#-------------------------------------------------

VERSION = 1.2.4
DEFINES += FV_APP_VERSION
FV_APP_VERSION = $$VERSION

QT += core gui concurrent widgets xml network quick

TARGET = StereoVisionToolkit
DEFINES += FV_APP_NAME
FV_APP_NAME = $$TARGET

TEMPLATE = app vcapp

CONFIG += warn_on
CONFIG += doc

# To use I3DR's pro stereo matcher
# add 'CONFIG+=include_pro' to build arguments
include_pro {
    message("Pro enabled")
    DEFINES += BUILD_PRO
}

RC_FILE = icon.rc

RESOURCES += \
    $$_PRO_FILE_PWD_/resources/qdarkstyle/style.qrc \
    $$_PRO_FILE_PWD_/resources/window/window.qrc

include($$_PRO_FILE_PWD_/resources/QtAwesome/QtAwesome.pri)

VPATH = $$_PRO_FILE_PWD_/src
INCLUDEPATH += $$_PRO_FILE_PWD_/src

include_pro {
    INCLUDEPATH += $$_PRO_FILE_PWD_/pro/src
}

SOURCES += \
    main.cpp\
    mainwindow.cpp \
    calibrationdialog.cpp \
    qdevicedialog.cpp \
    qdevicebutton.cpp \
    stereocalibrate.cpp \
    chessboard.cpp \
    calibrateconfirmdialog.cpp \
    calibratefromimagesdialog.cpp \
    abstractstereocamera.cpp \
    abstractstereomatcher.cpp \
    camerabasler.cpp \
    cameraopencv.cpp \
    stereocamerabasler.cpp \
    stereocamerabasler2.cpp \
    stereocameraopencv.cpp \
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

win32 {
    SOURCES += stereocameradeimos.cpp
}

include_pro {
    SOURCES += \
        $$_PRO_FILE_PWD_/pro/src/matcherwidgetjrsgm.cpp \
        $$_PRO_FILE_PWD_/pro/src/matcherjrsgm.cpp
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm2.cpp
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm3.cpp
}

HEADERS += \
    mainwindow.h \
    calibrationdialog.h \
    qdevicedialog.h \
    qdevicebutton.h \
    asmopencv.h \
    stereocalibrate.h \
    chessboard.h \
    calibrateconfirmdialog.h \
    calibratefromimagesdialog.h \
    abstractstereocamera.h \
    abstractstereomatcher.h \
    camerabasler.h \
    cameraopencv.h \
    stereocamerabasler.h \
    stereocamerabasler2.h \
    stereocameraopencv.h \
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

win32 {
    HEADERS += stereocameradeimos.h
}

include_pro {
    HEADERS += \
        $$_PRO_FILE_PWD_/pro/src/matcherwidgetjrsgm.h \
        $$_PRO_FILE_PWD_/pro/src/matcherjrsgm.h
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm2.h
        #$$_PRO_FILE_PWD_/pro/src/matcherjrsgm3.h
}

FORMS += \
    mainwindow.ui \
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

# For building in a release and debug in seperate folders
CONFIG(debug, debug|release) {
    DESTDIR = debug
    OBJECTS_DIR = .obj_debug
    MOC_DIR     = .moc_debug
}else {
    DESTDIR = release
    OBJECTS_DIR = .obj
    MOC_DIR     = .moc
}

# Error if running 32-bit system
# not all depencies are currently built to run on 32-bit
# remove this when you build the dependencies for 32-bit
contains(QT_ARCH, i386) {
    error(32-bit system detected. Cannot continue)
}

# Error if running unix system
# not all depencies are currently built to run on unix
# remove this when you build the dependencies for unix
unix {
    error(unix system detected. Cannot continue)
}

# Error if running mac system
# not all depencies are currently built to run on mac
# remove this when you build the dependencies for mac
macx {
    error(mac system detected. Cannot continue)
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
contains(QT_ARCH, i386) {
    #32-bit
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pylon/lib/Win32"
} else {
    #64-bit
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pylon/lib/x64"
}
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/pylon/include"

win32 {
    # Directshow class IDs
    LIBS += -lstrmiids
}

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

#TODO build 3rd party depencenies for 32-bit systems

win32 {
    # Define dlls to copy to build folder
    EXTRA_FILES += \
        $$files($$_PRO_FILE_PWD_/3rd_party/opengl/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/opencv/dep/310/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/qt/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/cuda/bin/*.dll, true)

    contains(QT_ARCH, i386) {
        #32-bit
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/3rd_party/pylon/bin/x86/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pylon/dep/Win32/*.dll, true)
    } else {
        #64-bit
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/3rd_party/pylon/bin/x64/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pylon/dep/x64/*.dll, true)
    }

    # Define drivers to copy to build folder
    win32 {
        EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rd_party/pylon/drivers/*.msi, true)
        EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rd_party/pylon/drivers/*.bat, true)
    }
    #TODO add pylon drivers for linux and mac

    include_pro {
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/bin/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/dep/*.DLL, true) \
            $$files($$_PRO_FILE_PWD_/pro/3rd_party/jr/lic/*.lic, true)
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/bin/*.dll, true) \
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/dep/*.dll, true) \
            #$$files($$_PRO_FILE_PWD_/pro/3rd_party/jr2/lic/*.lic, true)
    }

}

# Define additional files to copy to build folder
EXTRA_FILES += \
    $$_PRO_FILE_PWD_/resources/camera_serials.ini

# Get full output folder path
DEPLOY_FOLDER = $$OUT_PWD/$$DESTDIR

CONFIG( debug, debug|release ) {
    # DEBUG
    # Define debug only dlls to copy to build folder
    win32 {
        EXTRA_FILES += \
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
    # RELEASE
    # Define release only dlls to copy to build folder
    win32 {
        EXTRA_FILES += \
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
win32 {
    DEPLOY_COMMAND = windeployqt
}
macx {
    DEPLOY_COMMAND = macdeployqt
}
DEPLOY_TARGET = $$shell_quote($$shell_path($${DEPLOY_FOLDER}/$${TARGET}$${TARGET_CUSTOM_EXT}))
QMAKE_POST_LINK += $${DEPLOY_COMMAND} $${DEPLOY_TARGET}

# Copy 3rd party files to build folder
CONFIG += file_copies
COPIES += extraFiles
win32 {
    extraFiles.files = $${EXTRA_FILES}
}
extraFiles.path = $${DEPLOY_FOLDER}

# Install drivers
win32 {
    QMAKE_POST_LINK += && cd /d $${DEPLOY_FOLDER}
    QMAKE_POST_LINK += && install_drivers.bat
}

# Copy documentation to build folder
COPIES += helpDocs
helpDocs.files = $$files($$_PRO_FILE_PWD_/docs/help/*.html, true)
helpDocs.files += $$files($$_PRO_FILE_PWD_/docs/help/*.png, true)
helpDocs.path = $${DEPLOY_FOLDER}/docs/help

# Fervor autoupdater
!include("fervor/Fervor.pri") {
        error("Unable to include Fervor autoupdater.")
}

# Auto generate code documenation using doxygen
CONFIG( doc ){
    QMAKE_POST_LINK += && cd /d $${_PRO_FILE_PWD_} && doxygen
}

# Add clean command to remove all files from build directory
# use 'extraclean' in clean arguments to trigger this clean step
win32 {
    extraclean.commands = del /S /Q $$shell_quote($$shell_path($${DEPLOY_FOLDER})\*)
    extraclean.commands += del /S /Q $$shell_quote($$shell_path($${DEPLOY_FOLDER})\..\*)
}
unix {
    extraclean.commands = rm -r $$shell_quote($$shell_path($${DEPLOY_FOLDER})/*)
}
distclean.depends = extraclean
QMAKE_EXTRA_TARGETS += distclean extraclean
