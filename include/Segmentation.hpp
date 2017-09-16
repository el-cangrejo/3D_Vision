#ifndef SEGMENTATION_HPP_
#define SEGMENTATION_HPP_

#include "Open3DOR.hpp"

#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <algorithm>
#include <ctime>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

using std::cout;

class ImgSegmenter {
public:
  ImgSegmenter(void);
  ImgSegmenter(const cv::Mat &img);
  ImgSegmenter(cv::Mat &&img);
  ImgSegmenter(cv::Mat &&img, int, int, int, int);

  void estimateNormals(); // Estimates Normals for Every Point
  void printNormals();    // Prints Normals
  void detectNormalEdges(); // Detects Normal Edges and Print cos(thetas)
  void colorRegions(); // Colors the found Regions
  void writetoFile(std::string filename);
  void writetoMesh(Mesh &m, int step);
  void writetoMesh_impl(Mesh &m, int step);
  void writetoMesh(Mesh &m, int step, cv::Mat im, cv::Vec3b mask);

  virtual ~ImgSegmenter();

  cv::Mat orig_img;          // Original Input Image
  cv::Mat image;
  cv::Mat median_img;     // Median Blured Image
  cv::Mat norm_img;       // Image with Surface Normals for every pixel
  cv::Mat norm_color_img; // Image with Surface Normals mapped to RGB for every
                          // pixel
  cv::Mat norm_edge_img; // Image with painted Edges estimated from Surface
                         // Normals
  cv::Mat norm_bin_edge_img; // Image with painted Edges estimated from Surface
                             // Normals
  cv::Mat colored_img;       // Image with colored regions
private:
  int median_kernel; // Median Filter Kernel
  int normal_radius; // Radius of Normal Estimation Triangle
  int normal_step;   // Kernel to Print Normals
  int edge_radius;   // Radius of Surface Normal Edges Detection Window
  int num_regions;   // Number of regions found from color scheme
	int kernel_radius;

  clock_t begin, end;
  double elapsed_secs;

  cv::Rect crop;
  std::vector<cv::Point3f> normals; // Vector of Normals for every Point

  cv::Scalar getNextColor(int);
};

#endif // SEGMENTATION_HPP
