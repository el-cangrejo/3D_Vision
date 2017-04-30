/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#ifndef KINECTWIDGET_H
#define KINECTWIDGET_H

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

#include <QGLWidget>
#include <QMouseEvent>

#include <cmath>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "Mesh.hpp"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include "flextGL.h"

struct Vertex_
{
    float x, y;
    float u, v;
};
//
//class WithOpenGLBindings
//{
//private:
//    OpenGLBindings *bindings;
//protected:
//    WithOpenGLBindings() : bindings(0) {}
//    virtual ~WithOpenGLBindings() {}
//
//    virtual void onOpenGLBindingsChanged(OpenGLBindings *b) { }
//public:
//    void gl(OpenGLBindings *bindings)
//    {
//        this->bindings = bindings;
//        onOpenGLBindingsChanged(this->bindings);
//    }
//
//    OpenGLBindings *gl()
//    {
//        return bindings;
//    }
//};

template<size_t TBytesPerPixel, GLenum TInternalFormat, GLenum TFormat, GLenum TType>
struct ImageFormat
{
    static const size_t BytesPerPixel = TBytesPerPixel;
    static const GLenum InternalFormat = TInternalFormat;
    static const GLenum Format = TFormat;
    static const GLenum Type = TType;
};

typedef ImageFormat<1, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE> U8C1;
typedef ImageFormat<2, GL_R16I, GL_RED_INTEGER, GL_SHORT> S16C1;
typedef ImageFormat<2, GL_R16UI, GL_RED_INTEGER, GL_UNSIGNED_SHORT> U16C1;
typedef ImageFormat<4, GL_R32F, GL_RED, GL_FLOAT> F32C1;
typedef ImageFormat<8, GL_RG32F, GL_RG, GL_FLOAT> F32C2;
typedef ImageFormat<12, GL_RGB32F, GL_RGB, GL_FLOAT> F32C3;
typedef ImageFormat<4, GL_RGBA, GL_BGRA, GL_UNSIGNED_BYTE> F8C4;
typedef ImageFormat<16, GL_RGBA32F, GL_RGBA, GL_FLOAT> F32C4;

template<typename FormatT>
struct Texture //: public WithOpenGLBindings
{
protected:
    size_t bytes_per_pixel, height, width;

public:
    GLuint texture;
    unsigned char *data;
    size_t size;

    Texture() : bytes_per_pixel(FormatT::BytesPerPixel), height(0), width(0), texture(0), data(0), size(0)
    {
    }

    void bindToUnit(GLenum unit)
    {
        //gl()->glActiveTexture(unit);
				glActiveTexture(unit);
        glBindTexture(GL_TEXTURE_RECTANGLE, texture);
    }

    void allocate(size_t new_width, size_t new_height)
    {
        width = new_width;
        height = new_height;
        size = height * width * bytes_per_pixel;
        data = new unsigned char[size];

        glGenTextures(1, &texture);
        bindToUnit(GL_TEXTURE0);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, FormatT::InternalFormat, width, height, 0, FormatT::Format, FormatT::Type, 0);
				std::cout << FormatT::InternalFormat << " " << FormatT::Format << " " << FormatT::Type << "\n";
    }

    void deallocate()
    {
        glDeleteTextures(1, &texture);
        delete[] data;
    }

    void upload()
    {
        bindToUnit(GL_TEXTURE0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_RECTANGLE, /*level*/0, /*xoffset*/0, /*yoffset*/0, width, height, FormatT::Format, FormatT::Type, data);
    }

    void download()
    {
        downloadToBuffer(data);
    }

    void downloadToBuffer(unsigned char *data)
    {
        glReadPixels(0, 0, width, height, FormatT::Format, FormatT::Type, data);
    }

    void flipY()
    {
        flipYBuffer(data);
    }

    void flipYBuffer(unsigned char *data)
    {
        typedef unsigned char type;

        size_t linestep = width * bytes_per_pixel / sizeof(type);

        type *first_line = reinterpret_cast<type *>(data), *last_line = reinterpret_cast<type *>(data) + (height - 1) * linestep;

        for (size_t y = 0; y < height / 2; ++y)
        {
            for (size_t x = 0; x < linestep; ++x, ++first_line, ++last_line)
            {
                std::swap(*first_line, *last_line);
            }
            last_line -= 2 * linestep;
        }
    }

    libfreenect2::Frame *downloadToNewFrame()
    {
        libfreenect2::Frame *f = new libfreenect2::Frame(width, height, bytes_per_pixel);
        downloadToBuffer(f->data);
        flipYBuffer(f->data);

        return f;
    }
};

//struct ShaderProgram : public WithOpenGLBindings
//{
//    GLuint program, vertex_shader, fragment_shader;
//
//    char error_buffer[2048];
//
//    ShaderProgram() :
//        program(0),
//        vertex_shader(0),
//        fragment_shader(0)
//    {
//    }
//
//    void setVertexShader(const std::string& src)
//    {
//        const char* src_ = src.c_str();
//        int length_ = src.length();
//        vertex_shader = gl()->glCreateShader(GL_VERTEX_SHADER);
//        gl()->glShaderSource(vertex_shader, 1, &src_, &length_);
//    }
//
//    void setFragmentShader(const std::string& src)
//    {
//        const char* src_ = src.c_str();
//        int length_ = src.length();
//        fragment_shader = gl()->glCreateShader(GL_FRAGMENT_SHADER);
//        gl()->glShaderSource(fragment_shader, 1, &src_, &length_);
//    }
//
//    void build()
//    {
//        GLint status;
//
//        gl()->glCompileShader(vertex_shader);
//        gl()->glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);
//
//        if (status != GL_TRUE)
//        {
//            gl()->glGetShaderInfoLog(vertex_shader, sizeof(error_buffer), NULL, error_buffer);
//
//            std::cerr << "failed to compile vertex shader!" << std::endl << error_buffer << std::endl;
//        }
//
//        gl()->glCompileShader(fragment_shader);
//
//        gl()->glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
//        if (status != GL_TRUE)
//        {
//            gl()->glGetShaderInfoLog(fragment_shader, sizeof(error_buffer), NULL, error_buffer);
//
//            std::cerr << "failed to compile fragment shader!" << std::endl << error_buffer << std::endl;
//        }
//
//        program = gl()->glCreateProgram();
//        gl()->glAttachShader(program, vertex_shader);
//        gl()->glAttachShader(program, fragment_shader);
//
//        gl()->glLinkProgram(program);
//
//        gl()->glGetProgramiv(program, GL_LINK_STATUS, &status);
//
//        if (status != GL_TRUE)
//        {
//            gl()->glGetProgramInfoLog(program, sizeof(error_buffer), NULL, error_buffer);
//            std::cerr << "failed to link shader program!" << std::endl << error_buffer << std::endl;
//        }
//    }
//
//    GLint getAttributeLocation(const std::string& name)
//    {
//        return gl()->glGetAttribLocation(program, name.c_str());
//    }
//
//    void setUniform(const std::string& name, GLint value)
//    {
//        GLint idx = gl()->glGetUniformLocation(program, name.c_str());
//        if (idx == -1) return;
//
//        gl()->glUniform1i(idx, value);
//    }
//
//    void setUniform(const std::string& name, GLfloat value)
//    {
//        GLint idx = gl()->glGetUniformLocation(program, name.c_str());
//        if (idx == -1) return;
//
//        gl()->glUniform1f(idx, value);
//    }
//
//    void setUniformVector3(const std::string& name, GLfloat value[3])
//    {
//        GLint idx = gl()->glGetUniformLocation(program, name.c_str());
//        if (idx == -1) return;
//
//        gl()->glUniform3fv(idx, 1, value);
//    }
//
//    void setUniformMatrix3(const std::string& name, GLfloat value[9])
//    {
//        GLint idx = gl()->glGetUniformLocation(program, name.c_str());
//        if (idx == -1) return;
//
//        gl()->glUniformMatrix3fv(idx, 1, false, value);
//    }
//
//    void use()
//    {
//        gl()->glUseProgram(program);
//    }
//};

class KinectWidget : public QGLWidget {
public:
  explicit KinectWidget(QWidget *parent = 0);

  // GL Functions
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);
  void timerEvent(QTimerEvent *);
	Mesh generateMesh();
	void addFrame(std::string id, libfreenect2::Frame *frame);
public:
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  libfreenect2::SyncMultiFrameListener listener;
  libfreenect2::FrameMap frames;

  GLuint gl_depth_tex;
  GLuint gl_rgb_tex;

  std::string serial = "";

  bool viewer_enabled = true;
  bool enable_rgb = true;
  bool enable_depth = true;
  int deviceId = -1;
  size_t framemax = -1;
  // double freenect_angle(0);
  int got_frames, window;

  bool kinect_initialized;

  int width_, height_;

  cv::Mat depthMat;
private:
	Texture<F8C4> rgb;
	Texture<F32C1> ir;
	std::map<std::string,libfreenect2::Frame*> frames_;
};

#endif // KINECTWIDGET_H
