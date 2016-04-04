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

<<<<<<< HEAD
INCLUDEPATH += "/usr/include/pcl-1.7"\
               "/usr/include/eigen3"\
               "/usr/include/boost"\
               "include" \
               "src/cqtopencvviewergl"

INCLUDEPATH += "/usr/local/include/opencv2"
=======
INCLUDEPATH += "/usr/include/pcl-1.7" \
               "/usr/include/eigen3" \
               "/usr/include/boost" \
               "usr/local/include/opencv2" \
               "include"
>>>>>>> master

SOURCES += src/main.cpp\
           src/mainwindow.cpp \
           src/glwidget.cpp \
           src/Open3DOR.cpp \
<<<<<<< HEAD
           src/cqtopencvviewergl/cqtopencvviewergl.cpp
=======
           src/segmentation.cpp
>>>>>>> master

HEADERS  += include/mainwindow.h \
            include/glwidget.h \
            include/Open3DOR.hpp \
<<<<<<< HEAD
            src/cqtopencvviewergl/cqtopencvviewergl.h
=======
            include/segmentation.hpp
>>>>>>> master

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

<<<<<<< HEAD
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
=======
LIBS += -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui \
        -lopencv_imgcodecs \
>>>>>>> master

QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_DEBUG += -O1
