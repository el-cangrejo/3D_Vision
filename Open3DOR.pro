#-------------------------------------------------
#
# Project created by QtCreator 2016-02-28T13:46:52
#
#-------------------------------------------------

QT       += core gui opengl widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Open3DOR
TEMPLATE = app

CONFIG += c++11

INCLUDEPATH += "/usr/include/pcl-1.7"\
               "/usr/include/eigen3"\
               "/usr/include/boost"\
               "include" \
               "src/cqtopencvviewergl"

INCLUDEPATH += "/usr/include/pcl-1.7" \
               "/usr/include/eigen3" \
               "/usr/include/boost" \
               "usr/local/include/opencv2" \
               "include"

SOURCES += src/main.cpp\
           src/mainwindow.cpp \
           src/glwidget.cpp \
           src/Open3DOR.cpp \
           src/cqtopencvviewergl/cqtopencvviewergl.cpp \
           src/segmentation.cpp

HEADERS  += include/mainwindow.h \
            include/glwidget.h \
            include/Open3DOR.hpp \
            src/cqtopencvviewergl/cqtopencvviewergl.h \
            include/segmentation.hpp

FORMS    += mainwindow.ui

LIBS += -lglut -lGLU

LIBS += -lpcl_io\
        -lpcl_common \
        -lpcl_filters \
        -lpcl_kdtree \
        -lpcl_registration \
        -lpcl_features \
        -lpcl_segmentation \
        -lpcl_search

LIBS += -lboost_system \
        -lboost_filesystem

LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_flann
LIBS += -lopencv_imgcodecs


QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_DEBUG += -O1
