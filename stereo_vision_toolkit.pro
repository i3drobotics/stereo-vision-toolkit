#-------------------------------------------------
#
# Stereo Vision Toolkit
#
# Copyright I3D Robotics Ltd, 2020
# Authors: Josh Veitch-Michaelis, Ben Knight
#
#-------------------------------------------------

VERSION = 1.2.9
DEFINES += FV_APP_VERSION
FV_APP_VERSION = $$VERSION

QT += core gui concurrent widgets xml network quick

TARGET = StereoVisionToolkit
DEFINES += WITH_FERVOR
DEFINES += FV_APP_NAME
FV_APP_NAME = $$TARGET

TEMPLATE = app vcapp

CONFIG += warn_on
CONFIG += doc

# To use I3DRSGM
# add 'CONFIG+=WITH_I3DRSGM' to build arguments
WITH_I3DRSGM {
    message("I3DRSGM enabled")
    DEFINES += WITH_I3DRSGM
}

# To use Vimbda camera API (currently optional while being implimented)
# add 'CONFIG+=WITH_VIMBA' to build arguments
WITH_VIMBA {
    message("VIMBA enabled")
    DEFINES += WITH_VIMBA
}

RC_FILE = icon.rc

RESOURCES += \
    $$_PRO_FILE_PWD_/resources/qdarkstyle/style.qrc \
    $$_PRO_FILE_PWD_/resources/window/window.qrc

include($$_PRO_FILE_PWD_/resources/QtAwesome/QtAwesome.pri)

VPATH += $$_PRO_FILE_PWD_/src
VPATH += $$_PRO_FILE_PWD_/src/camera
VPATH += $$_PRO_FILE_PWD_/src/matcher
VPATH += $$_PRO_FILE_PWD_/src/calibrate
INCLUDEPATH += $$_PRO_FILE_PWD_/src
INCLUDEPATH += $$_PRO_FILE_PWD_/src/camera
INCLUDEPATH += $$_PRO_FILE_PWD_/src/matcher
INCLUDEPATH += $$_PRO_FILE_PWD_/src/calibrate

WITH_VIMBA {
    VPATH += $$_PRO_FILE_PWD_/src/vimba
    INCLUDEPATH += $$_PRO_FILE_PWD_/src/vimba
}

WITH_I3DRSGM {
    VPATH += $$_PRO_FILE_PWD_/i3drsgm/src
    INCLUDEPATH += $$_PRO_FILE_PWD_/i3drsgm/src
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

WITH_VIMBA {
    SOURCES += \
        stereocameravimba.cpp \
        ApiController.cpp \
        CameraObserver.cpp \
        FrameObserver.cpp
}

WITH_I3DRSGM {
    SOURCES += \
        matcherwidgeti3drsgm.cpp \
        qmatcheri3drsgm.cpp \
        matcheri3drsgm.cpp
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

WITH_VIMBA {
    HEADERS += \
        stereocameravimba.h \
        ApiController.h \
        CameraObserver.h \
        FrameObserver.h
}

WITH_I3DRSGM {
    HEADERS += \
        matcherwidgeti3drsgm.h \
        qmatcheri3drsgm.h \
        matcheri3drsgm.h
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

WITH_I3DRSGM {
    FORMS += matcherwidgeti3drsgm.ui
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

INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/opencv/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/boost/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/VTK/include/vtk-7.0"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/PCL/include/pcl-1.8"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/eigen"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/hidapi/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/tis/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/yaml-cpp/include"
INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/pylon/include"

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
LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/boost/lib"

WITH_VIMBA {
    # vimba library and include files
    INCLUDEPATH += "$$_PRO_FILE_PWD_/3rd_party/vimba/"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vimba/VimbaCPP/Lib/Win64/"
    LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/vimba/VimbaImageTransform/Lib/Win64/"
}

WITH_I3DRSGM {
    # I3DRSGM library and include files
    LIBS += -L"$$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/lib/PhobosIntegration" -lPhobosIntegration
    INCLUDEPATH += "$$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/include"
    DEPENDPATH += "$$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/dep"
}

# Basler library files
LIBS += -L"$$_PRO_FILE_PWD_/3rd_party/pylon/lib/x64"

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

win32 {
    # Define dlls to copy to build folder
    EXTRA_FILES += \
        $$files($$_PRO_FILE_PWD_/3rd_party/opengl/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/opencv/dep/310/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/cuda/bin/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/pylon/bin/x64/*.dll, true) \
        $$files($$_PRO_FILE_PWD_/3rd_party/pylon/dep/x64/*.dll, true) \
        $$_PRO_FILE_PWD_/3rd_party/hidapi/bin/Release/hidapi.dll

    CONFIG( debug, debug|release ) {
        # Debug only dlls
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/3rd_party/opencv/bin/debug/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pcl/bin/debug/*.dll, true) \
            $$_PRO_FILE_PWD_/3rd_party/png/libpng16d.dll \
            $$_PRO_FILE_PWD_/3rd_party/tbb/tbb_debug.dll \
            $$_PRO_FILE_PWD_/3rd_party/tis/bin/TIS_UDSHL11d_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/vtk/bin/debug/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/zlib/bin/debug/*.dll, true)
    } else {
        # Release only dlls
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/3rd_party/opencv/bin/release/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/pcl/bin/release/*.dll, true) \
            $$_PRO_FILE_PWD_/3rd_party/png/libpng16.dll \
            $$_PRO_FILE_PWD_/3rd_party/tbb/tbb.dll \
            $$_PRO_FILE_PWD_/3rd_party/tis/bin/TIS_UDSHL11_x64.dll \
            $$files($$_PRO_FILE_PWD_/3rd_party/vtk/bin/release/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/3rd_party/zlib/bin/release/*.dll, true)
    }

    # Define drivers to copy to build folder
    EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rd_party/pylon/drivers/*.msi, true)
    EXTRA_FILES += $$files($$_PRO_FILE_PWD_/3rd_party/pylon/drivers/*.bat, true)

    #TODO add pylon drivers for linux and mac
    WITH_I3DRSGM {
        EXTRA_FILES += \
            $$files($$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/bin/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/dep/*.dll, true) \
            $$files($$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/lic/*.lic, true) \
            $$files($$_PRO_FILE_PWD_/i3drsgm/3rd_party/i3dr/param/*.param, true)
    }

    WITH_VIMBA {
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rd_party/vimba/VimbaCPP/Bin/Win64/VimbaC.dll \
            $$_PRO_FILE_PWD_/3rd_party/vimba/VimbaImageTransform/Bin/Win64/VimbaImageTransform.dll

        CONFIG( debug, debug|release ) {
            # Debug only dlls
            EXTRA_FILES += $$_PRO_FILE_PWD_/3rd_party/vimba/VimbaCPP/Bin/Win64/VimbaCPPd.dll
        } else {
            EXTRA_FILES += $$_PRO_FILE_PWD_/3rd_party/vimba/VimbaCPP/Bin/Win64/VimbaCPP.dll
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
