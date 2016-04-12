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

class ImgSegmenter
{
public:
	ImgSegmenter(void);
	ImgSegmenter(const cv::Mat &img);
	ImgSegmenter(cv::Mat &&img);

    void estimateNormals();// Function to Estimate Normals for Every Point
    void printNormals();// Function to Print Normals
    void detectNormalEdges();// Function to Detect Normal Edges and Print cos(thetas)
    void colorRegions(); // Function to Color the found Regions
    void writetoFile(std::string filename);

	virtual ~ImgSegmenter();

	cv::Mat image;          		// Original Input Image
	cv::Mat median_img;     		// Median Blured Image
	cv::Mat norm_img;       		// Image with Surface Normals for every pixel
	cv::Mat norm_color_img; 		// Image with Surface Normals mapped to RGB for every pixel
	cv::Mat norm_edge_img;  		// Image with painted Edges estimated from Surface Normals
	cv::Mat norm_bin_edge_img; 	// Image with painted Edges estimated from Surface
	                       	// Normals
    cv::Mat colored_img;           	// Image with colored regions

	int median_kernel;     	// Median Filter Kernel
	int normal_radius;          	// Radius of Normal Estimation Triangle
	int normal_step;  	// Kernel to Print Normals
	int edge_radius; 	// Radius of Surface Normal Edges Detection Window
	int num_regions;         	// Number of regions found from color scheme

	clock_t begin, end;
	double elapsed_secs;

	std::vector<cv::Point3f> normals; // Vector of Normals for every Point
};

#endif // SEGMENTATION_HPP
