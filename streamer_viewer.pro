#-------------------------------------------------
#
# Streamer Viewer
#
# Copyright I3D Robotics Ltd, 2020
# Authors: Ben Knight
#
#-------------------------------------------------

VERSION = 0.0.1a

QT += core gui widgets network concurrent

#QT version > 5.12.4 uses openssl 1.1.1g
versionAtLeast(QT_VERSION, 5.12.4){
    message("Building with OpenSSL 1.1.1g")
} else {
    error("Use at least Qt version 5.12.4")
}

# Application name
TARGET = StreamerViewer

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

RC_FILE = $$_PRO_FILE_PWD_/resources/icon.rc

# Streamer
!include("modules/Streamer/Streamer.pri") {
    error("Unable to include Streamer.")
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

INCLUDEPATH += "$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/include"
INCLUDEPATH += $$_PRO_FILE_PWD_/modules/Streamer
VPATH += $$_PRO_FILE_PWD_/modules/Streamer

SOURCES += \
        streamerviewermain.cpp \
        streamerviewerwindow.cpp

HEADERS += \
        streamerviewerwindow.h

FORMS += \
        streamerviewerwindow.ui

# Define libaries
CONFIG(debug, debug|release) {
    # Define debug only libraries
    message("Debug mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/lib" -lopencv_world440d
}else {
    # Define release only libraries
    message("Release mode")
    LIBS += -L"$$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/lib" -lopencv_world440
}

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
        $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_videoio_ffmpeg440_64.dll

    CONFIG( debug, debug|release ) {
        # Debug only dlls
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_world440d.dll
    } else {
        # Release only dlls
        EXTRA_FILES += \
            $$_PRO_FILE_PWD_/3rdparty/opencv-4.4.0/opencv/build/x64/vc15/bin/opencv_world440.dll
    }
}

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
