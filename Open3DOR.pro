
QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Open3DOR
TEMPLATE = app

CONFIG += c++11

INCLUDEPATH += "/usr/include/pcl-1.7"\
               "/usr/include/eigen3"\
               "/usr/include/boost"


SOURCES += src/main.cpp\
           src/mainwindow.cpp \
           src/glwidget.cpp \
           src/Open3DOR.cpp

HEADERS  += include/mainwindow.h \
            include/glwidget.h \
            include/Open3DOR.hpp

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

QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_DEBUG += -O1
