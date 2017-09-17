#-------------------------------------------------
#
# Project created by QtCreator 2016-02-28T13:46:52
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Open3DOR
TEMPLATE = app

CONFIG += c++14

INCLUDEPATH += "/usr/local/include/pcl-1.8"\
               "/usr/include/eigen3"\
               "/usr/include/boost"\
							 "/usr/include/vl"\
               "include"\

INCLUDEPATH += "usr/local/include/opencv2"\
               "usr/local/include/libfreenect2"\
							 "/usr/include/libusb-1.0"\

SOURCES += src/main.cpp\
           src/mainwindow.cpp\
           src/GlWidget.cpp\
           src/Open3DOR.cpp\
           src/Segmentation.cpp\
           src/KinectWidget.cpp\
           src/ImgViewerWidget.cpp \
           src/MeshComponents.cpp \
           src/Mesh.cpp \

HEADERS +=	include/mainwindow.h \
            include/GlWidget.hpp \
            include/Open3DOR.hpp \
            include/Segmentation.hpp \
            include/KinectWidget.hpp \
            include/ImgViewerWidget.hpp \
            include/MeshComponents.hpp \
            include/Mesh.hpp \

FORMS    += mainwindow.ui

LIBS += -L/usr/local/lib

LIBS += -lglut -lGLU

LIBS += -lpcl_io\
        -lpcl_common \
        -lpcl_filters \
        -lpcl_kdtree \
        -lpcl_registration \
        -lpcl_features \
        -lpcl_segmentation \
				-lpcl_surface \
        -lpcl_search \

LIBS += -lboost_system \
        -lboost_filesystem

LIBS += -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui \
        -lopencv_video \
        -lopencv_imgcodecs

LIBS += -lfreenect2

LIBS += -lusb-1.0

LIBS += -lvl

QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_DEBUG += -O1
QMAKE_CXXFLAGS_DEBUG += -g

QMAKE_CC=clang
QMAKE_CXXFLAGS += -std=c++11
#QMAKE_CXX=clang++
