#include "segmentation.hpp"

ImgSegmenter::ImgSegmenter () {}

ImgSegmenter::ImgSegmenter (const cv::Mat &img) : image(img),
                                            median_kernel(1),
                                            normal_radius(1),
                                            normal_step(1),
                                            edge_radius(1),
                                            num_regions(0) {}

void ImgSegmenter::estimateNormals (void) {
  begin = clock();
  cout << "Estimation of Surface Normals begin \n";

  normals.reserve((median_img.rows - 2 * normal_radius - 1) *
                    (median_img.cols - 2 * normal_radius - 1));

  for (int i = normal_radius; i < image.rows - normal_radius; ++i) {
    for (int j = normal_radius; j < image.cols - normal_radius; ++j) {
      // Points a, b, c in the neighborhood of pixel (i, j)
      cv::Point3f a(i + normal_radius, j - normal_radius,
                (float)image.at<uchar>(i + normal_radius, j - normal_radius));
      cv::Point3f b(i + normal_radius, j + normal_radius,
                (float)image.at<uchar>(i + normal_radius, j + normal_radius));
      cv::Point3f c(i - normal_radius, j, (float)image.at<uchar>(i - normal_radius, j));
      cv::Point3f n;
      // Check if pixel is not a valid measurment
      if (a.z == 0 || b.z == 0 || c.z == 0) {
        n = cv::Point3f(0, 0, 0);
      } else {
        // Vectors for normal estimation
        cv::Point3f v1 = b - a;
        cv::Point3f v2 = c - a;
        // Normal is the cross product of v1 and v2
        n = v1.cross(v2);
        // Normalize vector
        float norm = sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
        // Check if length is zero
        if (norm == 0) {
          norm = 1;
        }
        n.x = n.x / norm;
        n.y = n.y / norm;
        n.z = n.z / norm;
      }
      normals.push_back(n);
    }
  }
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Estimation of Surface Normals end \nElapsed time: "
       << elapsed_secs << "\n";
}

void ImgSegmenter::printNormals (void) {
  begin = clock();
  cout << "Printing of Surface Normals begin \n";

  norm_img = median_img.clone();
  norm_color_img = median_img.clone();

  normal_step = 10;

  cvtColor(image, norm_color_img,
           CV_GRAY2RGB); // Make norm_color_img 3 channels

  for (int i = 0; i < norm_img.rows; ++i) {
    for (int j = 0; j < norm_img.cols; ++j) {
      // Calculate Index
      int index = i * norm_img.cols + j;
      cv::Point3f n = normals[index];
      // Map normals x, y, z to RGB
      norm_color_img.at<cv::Vec3b>(i, j)[0] = (255 * n.x);
      norm_color_img.at<cv::Vec3b>(i, j)[1] = (255 * n.y);
      norm_color_img.at<cv::Vec3b>(i, j)[2] = (255 * n.z);
      // Print normals with step kern
      if (i % normal_step == 0 && j % normal_step == 0) {
        cv::Point p4(25 * n.y, 25 * n.x);
        cv::Point p5(j, i);
        cv::Point p6 = p5 + p4;
        cv::arrowedLine(norm_img, p5, p6, cv::Scalar(255, 255, 255));
      }
    }
  }
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Printing of Surface Normals end \nElapsed time: " << elapsed_secs
       << "\n";
}

void ImgSegmenter::detectNormalEdges (void) {
  begin = clock();
  cout << "Detection of Surface Normal Edges begin \n";

  norm_edge_img = median_img.clone();


  norm_bin_edge_img = norm_edge_img.clone();
  for (int i = 0; i < norm_edge_img.rows; ++i) {
    for (int j = 0; j < norm_edge_img.cols; ++j) {
      if (i >= edge_radius && j >= edge_radius &&
          i < norm_edge_img.rows - edge_radius && j < norm_edge_img.cols - edge_radius) {
        // Calculate Index
        int index = i * norm_edge_img.cols + j;
        cv::Point3f n0 = normals[index];
        // Angles in the 8 directions
        float costheta = 0;   // Min angle
        float costhetan = 0;  // North
        float costhetas = 0;  // South
        float costhetaw = 0;  // West
        float costhetae = 0;  // East
        float costhetanw = 0; // North-West
        float costhetane = 0; // North-East
        float costhetasw = 0; // South-West
        float costhetase = 0; // South-East

        for (int count = 1; count <= edge_radius; ++count) {
          // North Direction
          cv::Point3f n = normals[(i - count) * norm_edge_img.cols + j];
          costhetan += n0.dot(n) / edge_radius;
          // South Direction
          cv::Point3f s = normals[(i + count) * norm_edge_img.cols + j];
          costhetas += n0.dot(s) / edge_radius;
          // West Direction
          cv::Point3f w = normals[i * norm_edge_img.cols + (j - count)];
          costhetaw += n0.dot(w) / edge_radius;
          // East Direction
          cv::Point3f e = normals[i * norm_edge_img.cols + (j + count)];
          costhetae += n0.dot(e) / edge_radius;
          // North West Direction
          cv::Point3f nw = normals[(i - count) * norm_edge_img.cols + (j - count)];
          costhetanw += n0.dot(nw) / edge_radius;
          // North East Direction
          cv::Point3f ne = normals[(i - count) * norm_edge_img.cols + (j + count)];
          costhetane += n0.dot(ne) / edge_radius;
          // South West Direction
          cv::Point3f sw = normals[(i + count) * norm_edge_img.cols + (j - count)];
          costhetasw += n0.dot(sw) / edge_radius;
          // South East Direction
          cv::Point3f se = normals[(i + count) * norm_edge_img.cols + (j + count)];
          costhetase += n0.dot(se) / edge_radius;
        }
        std::vector<float> thetas{costhetan,  costhetas,  costhetaw,
                                  costhetae,  costhetanw, costhetane,
                                  costhetasw, costhetase};

        costheta = *std::min_element(thetas.begin(), thetas.end());
        // Print cos(thetas)
        norm_edge_img.at<uchar>(i, j) = costheta * 255;
        // Print binary cos(thetas)
        if (costheta > 0.93) {
          norm_bin_edge_img.at<uchar>(i, j) = 255;
        } else {
          norm_bin_edge_img.at<uchar>(i, j) = 0;
        }
      }
    }
  }



  cv::medianBlur(norm_bin_edge_img, norm_bin_edge_img, 5);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Detection of Surface Normal Edges end \nElapsed time: "
       << elapsed_secs << "\n";
}

void ImgSegmenter::colorRegions (void) {
  begin = clock();
  cout << "Region Growing begin \n";

  std::vector<cv::Scalar> colors{cv::Scalar(0, 0, 255),   cv::Scalar(0, 255, 0),
                             cv::Scalar(255, 0, 0),   cv::Scalar(255, 0, 255),
                             cv::Scalar(0, 255, 255), cv::Scalar(255, 255, 0)};
  
  for (int i = 0; i < colored_img.rows; ++i) {
    for (int j = 0; j < colored_img.cols; ++j) {
      cv::Point seed;
      if (colored_img.at<cv::Vec3b>(i, j) == cv::Vec3b(255, 255, 255)) {
        ++num_regions;
        seed = cv::Point(j, i);
        cv::floodFill(colored_img, seed, colors[num_regions % 6]);
      }
    }
  }
  
  int ke = 7;
  for (int r = 0; r < colored_img.rows; ++r) {
    for (int c = 0; c < colored_img.cols; ++c) {
      if (colored_img.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0)) {
        cv::Point min_distp = cv::Point(0, 0);
        float min_dist = 3;
        for (int i = -ke/2; i <= ke/2; ++i) {
          for (int j = -ke/2; j < ke/2; ++j) {
            int image_r = std::min(std::max(r + i, 0), (int)(colored_img.rows - 1));
            int image_c = std::min(std::max(c + j, 0), (int)(colored_img.cols - 1));
            if (colored_img.at<cv::Vec3b>(image_r, image_c) != cv::Vec3b(0, 0, 0)) {
              float depth_dif = abs((int)median_img.at<uchar>(image_r, image_c) -
                                (int)median_img.at<uchar>(r, c));
              float euc_dist = sqrt(pow(i, 2) + pow(j, 2) + pow(depth_dif, 2));
              if (euc_dist < min_dist) {
                min_dist = euc_dist;
                min_distp = cv::Point(image_c, image_r);
              }
            }
          }
        }
        if (min_distp != cv::Point(0, 0)) {
          colored_img.at<cv::Vec3b>(r, c) = colored_img.at<cv::Vec3b>(min_distp);
        }
      } 
    }
  }

  cout << "Regions found: " << num_regions << "\n";
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Region Growing end \n";
  cout << "Elapsed time: " << elapsed_secs << "\n";
}

void ImgSegmenter::writetoFile (std::string filename) { 
  std::ofstream myfile;
  myfile.open (filename + ".obj");
  
  int count_i(0);
  int count_j(0);

  for (int i = 0; i < colored_img.rows; ++i) {
    for (int j = 0; j < colored_img.cols; ++j) {
      if ((i % 2 == 0) && (j % 2 == 0)) {
        //cout << "I = " << i << "J = " << j << "\n";
        myfile << "v " << ((colored_img.cols / 2.0) - j) / colored_img.cols << " "
        << ((colored_img.rows / 2.0) - i) / colored_img.rows << " "
        << (int)median_img.at<uchar>(i, j) / 255. << "\n";
        myfile << "c " << colored_img.at<cv::Vec3b>(i, j)[0] / 255 << " "
        << colored_img.at<cv::Vec3b>(i, j)[1] / 255 << " "
        << colored_img.at<cv::Vec3b>(i, j)[2] / 255 << "\n";
        if ( (i <= colored_img.rows - 2) && (j <= colored_img.cols - 4)) {
          myfile << "f " << (count_i*(colored_img.cols/2)+j/2) << " "
                         << (count_i*(colored_img.cols/2)+(j/2+1)) << " "
                         << ((count_i+1)*(colored_img.cols/2)+(j/2+1))
                         << "\n";
          myfile << "f " << (count_i*(colored_img.cols/2)+j/2) << " "
                         << ((count_i+1)*(colored_img.cols/2)+j/2) << " "
                         << ((count_i+1)*(colored_img.cols/2)+(j/2+1))
                         << "\n";
        }
      } 
      //if (j % 2 == 0) ++count_j;
    }
    if (i % 2 == 0) ++count_i;
  }
  cout << "count_i = " << count_i << " count_j = " << count_j << "\n";
  cout << colored_img.rows << " " << (int)colored_img.cols/2 << "\n";
  myfile.close();
}

ImgSegmenter::~ImgSegmenter () {}

int prepare_segmentation(cv::Mat &img) {

  ImgSegmenter segmenter(img);

  // Median Filter for Noise Reduction
  cv::medianBlur(segmenter.image, segmenter.median_img, segmenter.median_kernel);

  // Estimation of Surface Normals
  segmenter.estimateNormals();

  // Prints Normal Vector at every pixel and Map xyz to RGB

  cv::Rect crop(segmenter.normal_radius, segmenter.normal_radius, segmenter.norm_color_img.cols - 2 * segmenter.normal_radius,
            segmenter.norm_color_img.rows - 2 * segmenter.normal_radius); // Crop image according to normal_radius

  segmenter.norm_color_img = segmenter.norm_color_img(crop);
  segmenter.norm_img = segmenter.norm_img(crop);

  segmenter.printNormals();

  // Detection of Surface Normal Edges

  segmenter.norm_edge_img = segmenter.norm_edge_img(crop);
  segmenter.detectNormalEdges();
  crop = cv::Rect(segmenter.edge_radius, segmenter.edge_radius,
              segmenter.norm_edge_img.cols - 2 * segmenter.edge_radius,
              segmenter.norm_edge_img.rows - 2 * segmenter.edge_radius);
  segmenter.norm_edge_img = segmenter.norm_edge_img(crop);
  segmenter.norm_bin_edge_img = segmenter.norm_bin_edge_img(crop);

  cvtColor(segmenter.norm_bin_edge_img, segmenter.colored_img, CV_GRAY2RGB);
  
  segmenter.colorRegions();

  return 0;
}
