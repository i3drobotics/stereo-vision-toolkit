#-------------------------------------------------
#
# Stereo Vision Toolkit
#
# Copyright I3D Robotics Ltd, 2020
# Authors: Josh Veitch-Michaelis, Ben Knight
#
#-------------------------------------------------

VERSION = 1.3.1a.10
DEFINES += FV_APP_VERSION
FV_APP_VERSION = $$VERSION

QT += core gui concurrent widgets xml network quick serialport

#QT version > 5.12.4 uses openssl 1.1.1g
versionAtLeast(QT_VERSION, 5.12.4){
    message("Building with OpenSSL 1.1.1g")
} else {
    error("Use at least Qt version 5.12.4")
}

# Application name
TARGET = StereoVisionToolkit
# Application type
TEMPLATE = app vcapp

CONFIG += warn_on
CONFIG += doc
CONFIG -= debug_and_release
CONFIG -= debug_and_release_target

# Define if doing development build
# !! ONLY USE WHEN ON DEVELOPMENT BRANCH !!
# !! MAKE SURE TO REMOVE THIS BUILD OPTION WHEN DOING RELEASE !!
DEV_BRANCH {
    message("Development build")
    message("!! MAKE SURE TO REMOVE [CONFIG+=DEV_BRANCH] BUILD OPTION WHEN DOING MASTER RELEASE !!")
    DEFINES += DEV_BRANCH

    CONFIG(debug, debug|release) { # debug
    }else { # release
        CONFIG += console
    }
}

# Setup FERVOR defines
DEFINES += WITH_FERVOR
DEFINES += FV_APP_NAME
FV_APP_NAME = $$TARGET

# To disable StereoStreamer
# comment out this line 'CONFIG+=WITH_STEREOSTREAMER'
CONFIG+=WITH_STEREO_STREAMER
WITH_STEREO_STREAMER {
    message("StereoStreamer enabled")
    DEFINES += WITH_STEREO_STREAMER
}

# To use I3DRSGM
# add 'CONFIG+=WITH_I3DRSGM' to build arguments
WITH_I3DRSGM {
    message("I3DRSGM enabled")
    DEFINES += WITH_I3DRSGM
}

# To use CUDA
# add 'CONFIG+=WITH_CUDA' to build arguments
WITH_CUDA {
    message("CUDA enabled")
    DEFINES += WITH_CUDA
}

# To use Vimbda camera API (currently optional while being implimented)
# add 'CONFIG+=WITH_VIMBA' to build arguments
# Vimba remains optional as there is an issue with debugging when enabled
# so it is useful to be able to turn it off for debug build
WITH_VIMBA {
    message("VIMBA enabled")
    DEFINES += WITH_VIMBA
}

# Define resources
RC_FILE = $$_PRO_FILE_PWD_/resources/icon.rc

RESOURCES += \
    $$_PRO_FILE_PWD_/resources/qdarkstyle/style.qrc \
    $$_PRO_FILE_PWD_/resources/window/window.qrc

# ---Include modules---
# QtAwesome
!include(modules/QtAwesome/QtAwesome.pri) {
    error("Unable to include QtAwesome.")
}

# Fervor autoupdater
!include("modules/fervor/Fervor.pri") {
        error("Unable to include Fervor autoupdater.")
}

# StereoStreamer
!include("modules/StereoStreamer/StereoStreamer.pri") {
    error("Unable to include StereoStreamer.")
}
# ---------------------

# Define search paths for files
VPATH += $$_PRO_FILE_PWD_/src
VPATH += $$_PRO_FILE_PWD_/src/camera
VPATH += $$_PRO_FILE_PWD_/src/matcher
VPATH += $$_PRO_FILE_PWD_/src/calibrate
VPATH += $$_PRO_FILE_PWD_/src/camera/widgets
VPATH += $$_PRO_FILE_PWD_/src/matcher/widgets
VPATH += $$_PRO_FILE_PWD_/src/camera/virtualcam
VPATH += $$_PRO_FILE_PWD_/src/camera/cameracontrol
VPATH += $$_PRO_FILE_PWD_/src/detection
INCLUDEPATH += $$_PRO_FILE_PWD_/src
INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera
INCLUDEPATH += $$_PRO_FILE_PWD_/src/matcher
INCLUDEPATH += $$_PRO_FILE_PWD_/src/calibrate
INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera/widgets
INCLUDEPATH += $$_PRO_FILE_PWD_/src/matcher/widgets
INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera/virtualcam
INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera/cameracontrol
INCLUDEPATH += $$_PRO_FILE_PWD_/src/detection

WITH_VIMBA {
    VPATH += $$_PRO_FILE_PWD_/src/camera/vimba
    INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera/vimba
}

# Define source files
SOURCES += \
    svtkmain.cpp \
    svtkwindow.cpp \
    arduinocommscameracontrol.cpp \
    aboutdialog.cpp \
    calibrationdialog.cpp \
    abstractarduinocoms.cpp \
    detectoropencv.cpp \
    detectorsetupdialog.cpp \
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
    cameraimagingsource.cpp \
    virtualcam.cpp
# Deimos source file is windows only for directshow
win32 {
    SOURCES += stereocameradeimos.cpp
}
# Optional vimba source files (as currently in development)
WITH_VIMBA {
    SOURCES += \
        camera/cameravimba.cpp \
        stereocameravimba.cpp
}
# Optional I3RSGM matcher source files
WITH_I3DRSGM {
    SOURCES += \
        matcherwidgeti3drsgm.cpp \
        matcheri3drsgm.cpp
}

# Define header files
HEADERS += \
    svtkwindow.h \
    arduinocommscameracontrol.h \
    aboutdialog.h \
    calibrationdialog.h \
    asmopencv.h \
    abstractarduinocoms.h \
    cvsupport.h \
    pclsupport.h \
    pylonsupport.h \
    boundingbox.h \
    detectoropencv.h \
    detectorsetupdialog.h \
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
    stereocameratis.h \
    stereocamerafromvideo.h \
    stereocamerasupport.h \
    matcheropencvblock.h \
    matcherwidgetopencvblock.h \
    matcheropencvsgbm.h \
    matcherwidget.h \
    matcherwidgetopencvsgbm.h \
    disparityviewer.h \
    paramfile.h \
    cameradisplaywidget.h \
    cameraimagingsource.h \
    virtualcam.h
# Deimos header file is windows only for directshow
win32 {
    HEADERS += stereocameradeimos.h
}
# Optional vimba header files (as currently in development)
WITH_VIMBA {
    HEADERS += \
        camera/cameravimba.h \
        stereocameravimba.h
}
# Optional I3DRSGM matcher header files
WITH_I3DRSGM {
    HEADERS += \
        matcherwidgeti3drsgm.h \
        matcheri3drsgm.h
}
# Define application window forms
FORMS += \
    svtkwindow.ui \
    aboutdialog.ui \
    calibrationdialog.ui \
    calibrateconfirmdialog.ui \
    calibratefromimagesdialog.ui \
    matcherwidgetopencvblock.ui \
    matcherwidgetopencvsgbm.ui \
    disparityviewer.ui \
    cameradisplaywidget.ui \
    aboutdialog.ui \
    detectorsetupdialog.ui
# Optional I3DRSGM window form
WITH_I3DRSGM {
    FORMS += matcherwidgeti3drsgm.ui
}

# For building in a release and debug in seperate folders
CONFIG(debug, debug|release) { #debug
    DESTDIR = $$TARGET/debug
    OBJECTS_DIR = $$TARGET/.obj_debug
    MOC_DIR     = $$TARGET/.moc_debug
}else {
    DESTDIR = $$TARGET/release
    OBJECTS_DIR = $$TARGET/.obj
    MOC_DIR     = $$TARGET/.moc
}
RCC_DIR = $$TARGET/.qrc
UI_DIR = $$TARGET/.ui

# Error if running 32-bit system
# depencies are built to run on 64-bit system
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

# Define include folders for libraries
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/boost-1.66.0/boost_1_66_0"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/VTK/include/vtk-7.0"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/PCL/include/pcl-1.8"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/eigen"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/hidapi/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/tis/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/yaml-cpp/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/pylon/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/dshow/include"

# Define libaries
CONFIG(debug, debug|release) {
    # Define debug only libraries
    message("Debug mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/hidapi/lib/debug" -lhidapi
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/tis/lib/debug" -lTIS_UDSHL11d_x64
    LIBS += -lpcl_visualization_debug -lpcl_io_debug -lpcl_common_debug -lpcl_filters_debug
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/lib" -lopencv_world440d
}else {
    # Define release only libraries
    message("Release mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/pcl/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/vtk/lib"
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/hidapi/lib/release" -lhidapi
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/tis/lib/release" -lTIS_UDSHL11_x64
    LIBS += -lpcl_visualization_release -lpcl_io_release -lpcl_common_release -lpcl_filters_release
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/lib" -lopencv_world440
}

# Define libraries
LIBS += -lvtkCommonCore-7.0 -lvtkCommonDataModel-7.0 -lvtkGUISupportQt-7.0 -lvtkViewsQt-7.0 -lvtkViewsCore-7.0 -lvtkRenderingQt-7.0  -lvtkCommonMath-7.0 -lvtkRenderingCore-7.0 -lvtkIOCore-7.0
LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/boost-1.66.0/boost_1_66_0/stage/lib"

WITH_VIMBA {
    # Vimba library and include files
    INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/vimba/"
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCPP/Lib/Win64/" -lVimbaCPP
}

WITH_I3DRSGM {
    # I3DRSGM library and include files
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/i3drsgm-1.0.4/lib/" -lI3DRSGM
    INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/i3drsgm-1.0.4/include"
    # PhobosIntegration library and include files (required for I3DRSGM)
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/phobosIntegration-1.0.54/lib/PhobosIntegration" -lPhobosIntegration
    INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/phobosIntegration-1.0.54/include"
}

# Basler library files
LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/pylon/lib/x64"

win32 {
    # Directshow libraries
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/dshow/lib/x64" -lstrmbasd -lstrmbase
    # Directshow class IDs
    LIBS += -lstrmiids
}

# Define fonts
DISTFILES += $$_PRO_FILE_PWD_/resources/fonts/fontawesome-webfont.ttf

# Define target
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
    # Define dlls to copy to build folder (windows only)
    EXTRA_FILES += \
        $$files($$_PRO_FILE_PWD_/3rdparty/opengl/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rdparty/cuda/bin/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rdparty/pylon/bin/x64/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rdparty/pylon/dep/x64/*.dll, true) \
        $$_PRO_FILE_PWD_/3rdparty/hidapi/bin/Release/hidapi.dll \
        $$_PRO_FILE_PWD_/3rdparty/tbb/tbb.dll \
        $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_videoio_ffmpeg440_64.dll \
        #$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/openh264-1.8.0-win64.dll \
        $$files($$_PRO_FILE_PWD_/3rdparty/openssl-1.1.1g/Win64/bin/*.dll, true)

    CONFIG( debug, debug|release ) {
        # Debug only dlls
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_world440d.dll \
            $$files($$_PRO_FILE_PWD_/3rdparty/pcl/bin/debug/*.dll, true) \
            $$_PRO_FILE_PWD_/3rdparty/png/libpng16d.dll \
            $$_PRO_FILE_PWD_/3rdparty/tbb/tbb_debug.dll \
            $$_PRO_FILE_PWD_/3rdparty/tis/bin/TIS_UDSHL11d_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rdparty/vtk/bin/debug/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rdparty/zlib/bin/debug/*.dll, true)
    } else {
        # Release only dlls
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_world440.dll \
            $$files($$_PRO_FILE_PWD_/3rdparty/pcl/bin/release/*.dll, true) \
            $$_PRO_FILE_PWD_/3rdparty/png/libpng16.dll \
            $$_PRO_FILE_PWD_/3rdparty/tis/bin/TIS_UDSHL11_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rdparty/vtk/bin/release/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rdparty/zlib/bin/release/*.dll, true)
    }

    # Define drivers to copy to build folder
    EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rdparty/pylon/drivers/*.msi, true)
    EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rdparty/pylon/drivers/*.bat, true)

    # I3DRSGM dlls
    WITH_I3DRSGM {
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/i3drsgm-1.0.4/bin/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/i3drsgm-1.0.4/bin/*.param, true) \
            $$files($$_PRO_FILE_PWD_/3rdparty/i3drsgm-1.0.4/i3drsgm/phobosIntegration-1.0.54/bin/*.dll, true)
    }

    # Vimba dlls
    WITH_VIMBA {
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaC/Bin/Win64/VimbaC.dll \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCLConfigTL/Bin/Win64/clallserial.dll \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCLConfigTL/Bin/Win64/VimbaCLConfigTL.cti \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCLConfigTL/Bin/Win64/VimbaCLConfigTL.xml \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaUSBTL/Bin/Win64/VimbaUSBTL.cti \
            $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaUSBTL/Bin/Win64/VimbaUSBTL.xml \

        CONFIG( debug, debug|release ) {
            # Debug only dlls
            # TODO find out why CPP is needed with CPPd (causing debug issues)
            EXTRA_FILES += \
                $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCPP/Bin/Win64/VimbaCPP.dll \
                $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCPP/Bin/Win64/VimbaCPPd.dll
        } else {
            EXTRA_FILES += $$_PRO_FILE_PWD_/3rdparty/vimba/VimbaCPP/Bin/Win64/VimbaCPP.dll
        }
    }
}

# Define additional files to copy to build folder
EXTRA_FILES += \
    $$_PRO_FILE_PWD_/resources/camera_serials.ini

# Get full output folder path
DEPLOY_FOLDER = $$OUT_PWD/$$DESTDIR

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
    message("Copying dependencies")
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
helpDocs.files = $$files($$_PRO_FILE_PWD_/docs/app/*.pdf, true)
helpDocs.path = $${DEPLOY_FOLDER}/docs/help

# Copy example ml to build folder
COPIES += mlExamples
mlExamples.files = $$files($$_PRO_FILE_PWD_/resources/ml/coco/*, true)
mlExamples.path = $${DEPLOY_FOLDER}/ml/coco

# Auto generate code documenation using doxygen
CONFIG( doc ){
    QMAKE_POST_LINK += && cd /d $${_PRO_FILE_PWD_} && doxygen -u
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
