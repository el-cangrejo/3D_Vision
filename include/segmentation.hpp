#ifndef SEGMENTATION_HPP_
#define SEGMENTATION_HPP_

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui_c.h"

#include <stdio.h>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <initializer_list>

using std::cout;
using namespace cv;

// Function to Estimate Normals for Every Point
void estimate_normals(const Mat &, const int, std::vector<Point3f> &);

// Function to Print Normals
void print_normals(Mat &, Mat &, const std::vector<Point3f>, const int,
                   const int);

// Function to Detect Normal Edges and Print cos(thetas)
void detect_normal_edges(Mat &, Mat &, const std::vector<Point3f>, const int,
                         const int);

// Function to Color the found Regions
void color_regions(const Mat &, Mat &, int &);

//
void write_tofile(const Mat &, const Mat &);

Mat image;          // Original Input Image
Mat median_img;     // Median Blured Image
Mat norm_img;       // Image with Surface Normals for every pixel
Mat norm_color_img; // Image with Surface Normals mapped to RGB for every pixel
Mat norm_edge_img;  // Image with painted Edges estimated from Surface Normals
Mat norm_bin_edge_img; // Image with painted Edges estimated from Surface
                       // Normals
Mat colored;           // Image with colored regions

int kernel_size;     // Median Filter Kernel
int radius;          // Radius of Normal Estimation Triangle
int kernel_normals;  // Kernel to Print Normals
int kernel_normedge; // Radius of Surface Normal Edges Detection Window
int regions;         // Number of regions found from color scheme

clock_t begin, end;
double elapsed_secs;

std::vector<Point3f> normals; // Vector of Normals for every Point

#endif // SEGMENTATION_HPP