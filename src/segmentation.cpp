#include "segmentation.hpp"

ImgSegmenter::ImgSegmenter () {}

ImgSegmenter::ImgSegmenter (cv::Mat &img) image(img), 
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

  for (int i = normal_radius; i < img.rows - normal_radius; ++i) {
    for (int j = normal_radius; j < img.cols - normal_radius; ++j) {
      // Points a, b, c in the neighborhood of pixel (i, j)
      Point3f a(i + normal_radius, j - normal_radius,
                (float)img.at<uchar>(i + normal_radius, j - normal_radius));
      Point3f b(i + normal_radius, j + normal_radius,
                (float)img.at<uchar>(i + normal_radius, j + normal_radius));
      Point3f c(i - normal_radius, j, (float)img.at<uchar>(i - normal_radius, j));
      Point3f n;
      // Check if pixel is not a valid measurment
      if (a.z == 0 || b.z == 0 || c.z == 0) {
        n = Point3f(0, 0, 0);
      } else {
        // Vectors for normal estimation
        Point3f v1 = b - a;
        Point3f v2 = c - a;
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

  kernel_normals = 10;

  cvtColor(image, norm_color_img,
           CV_GRAY2RGB); // Make norm_color_img 3 channels

  for (int i = 0; i < arrowed_dst.rows; ++i) {
    for (int j = 0; j < arrowed_dst.cols; ++j) {
      // Calculate Index
      int index = i * arrowed_dst.cols + j;
      Point3f n = normals[index];
      // Map normals x, y, z to RGB
      color_dst.at<Vec3b>(i, j)[0] = (255 * n.x);
      color_dst.at<Vec3b>(i, j)[1] = (255 * n.y);
      color_dst.at<Vec3b>(i, j)[2] = (255 * n.z);
      // Print normals with step kern
      if (i % kernel == 0 && j % kernel == 0) {
        Point p4(25 * n.y, 25 * n.x);
        Point p5(j, i);
        Point p6 = p5 + p4;
        arrowedLine(arrowed_dst, p5, p6, Scalar(255, 255, 255));
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
  for (int i = 0; i < dst.rows; ++i) {
    for (int j = 0; j < dst.cols; ++j) {
      if (i >= edge_radius && j >= edge_radius &&
          i < dst.rows - edge_radius && j < dst.cols - edge_radius) {
        // Calculate Index
        int index = i * dst.cols + j;
        Point3f n0 = norm[index];
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
          Point3f n = normals[(i - count) * dst.cols + j];
          costhetan += n0.dot(n) / edge_radius;
          // South Direction
          Point3f s = normals[(i + count) * dst.cols + j];
          costhetas += n0.dot(s) / edge_radius;
          // West Direction
          Point3f w = normals[i * dst.cols + (j - count)];
          costhetaw += n0.dot(w) / edge_radius;
          // East Direction
          Point3f e = normals[i * dst.cols + (j + count)];
          costhetae += n0.dot(e) / edge_radius;
          // North West Direction
          Point3f nw = normals[(i - count) * dst.cols + (j - count)];
          costhetanw += n0.dot(nw) / edge_radius;
          // North East Direction
          Point3f ne = normals[(i - count) * dst.cols + (j + count)];
          costhetane += n0.dot(ne) / edge_radius;
          // South West Direction
          Point3f sw = normals[(i + count) * dst.cols + (j - count)];
          costhetasw += n0.dot(sw) / edge_radius;
          // South East Direction
          Point3f se = normals[(i + count) * dst.cols + (j + count)];
          costhetase += n0.dot(se) / edge_radius;
        }
        std::vector<float> thetas{costhetan,  costhetas,  costhetaw,
                                  costhetae,  costhetanw, costhetane,
                                  costhetasw, costhetase};

        costheta = *std::min_element(thetas.begin(), thetas.end());
        // Print cos(thetas)
        dst.at<uchar>(i, j) = costheta * 255;
        // Print binary cos(thetas)
        if (costheta > 0.93) {
          dst_bin.at<uchar>(i, j) = 255;
        } else {
          dst_bin.at<uchar>(i, j) = 0;
        }
      }
    }
  }



  medianBlur(norm_bin_edge_img, norm_bin_edge_img, 5);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Detection of Surface Normal Edges end \nElapsed time: "
       << elapsed_secs << "\n";
}

void ImgSegmenter::colorRegions (void) {
  std::vector<Scalar> colors{Scalar(0, 0, 255),   Scalar(0, 255, 0),
                             Scalar(255, 0, 0),   Scalar(255, 0, 255),
                             Scalar(0, 255, 255), Scalar(255, 255, 0)};  
  
  for (int i = 0; i < dst.rows; ++i) {
    for (int j = 0; j < dst.cols; ++j) {
      Point seed;
      if (dst.at<Vec3b>(i, j) == Vec3b(255, 255, 255)) {
        ++regions;
        seed = Point(j, i);
        floodFill(dst, seed, colors[regions % 6]); 
      }
    }
  }
  
  int ke = 7;
  for (int r = 0; r < dst.rows; ++r) {
    for (int c = 0; c < dst.cols; ++c) {
      if (dst.at<Vec3b>(r, c) == Vec3b(0, 0, 0)) {
        Point min_distp = Point(0, 0);
        float min_dist = 3;
        for (int i = -ke/2; i <= ke/2; ++i) {
          for (int j = -ke/2; j < ke/2; ++j) {
            int image_r = std::min(std::max(r + i, 0), (int)(dst.rows - 1));
            int image_c = std::min(std::max(c + j, 0), (int)(dst.cols - 1));
            if (dst.at<Vec3b>(image_r, image_c) != Vec3b(0, 0, 0)) { 
              float depth_dif = abs((int)median.at<uchar>(image_r, image_c) - 
                                (int)median.at<uchar>(r, c));
              float euc_dist = sqrt(pow(i, 2) + pow(j, 2) + pow(depth_dif, 2));
              if (euc_dist < min_dist) {
                min_dist = euc_dist;
                min_distp = Point(image_c, image_r);
              }
            }
          }
        }
        if (min_distp != Point(0, 0)) {
          dst.at<Vec3b>(r, c) = dst.at<Vec3b>(min_distp);
        }
      } 
    }
  }
}

void ImgSegmenter::writetoFile (std::string filename) { 
  std::ofstream myfile;
  myfile.open (filename + ".obj");
  
  int count_i(0);
  int count_j(0);

  for (int i = 0; i < colored.rows; ++i) {
    for (int j = 0; j < colored.cols; ++j) {
      if ((i % 2 == 0) && (j % 2 == 0)) {
        //cout << "I = " << i << "J = " << j << "\n";
        myfile << "v " << ((colored.cols / 2.0) - j) / colored.cols << " " 
        << ((colored.rows / 2.0) - i) / colored.rows << " " 
        << (int)median.at<uchar>(i, j) / 255. << "\n";
        myfile << "c " << colored.at<Vec3b>(i, j)[0] / 255 << " "
        << colored.at<Vec3b>(i, j)[1] / 255 << " "
        << colored.at<Vec3b>(i, j)[2] / 255 << "\n";
        if ( (i <= colored.rows - 2) && (j <= colored.cols - 4)) {
          myfile << "f " << (count_i*(colored.cols/2)+j/2) << " "
                         << (count_i*(colored.cols/2)+(j/2+1)) << " "
                         << ((count_i+1)*(colored.cols/2)+(j/2+1)) 
                         << "\n";
          myfile << "f " << (count_i*(colored.cols/2)+j/2) << " "
                         << ((count_i+1)*(colored.cols/2)+j/2) << " "
                         << ((count_i+1)*(colored.cols/2)+(j/2+1)) 
                         << "\n";
        }
      } 
      //if (j % 2 == 0) ++count_j;
    }
    if (i % 2 == 0) ++count_i;
  }
  cout << "count_i = " << count_i << " count_j = " << count_j << "\n";
  cout << colored.rows << " " << (int)colored.cols/2 << "\n";
  myfile.close();
}

ImgSegmenter::~ImgSegmenter () {}

int prepare_segmentation(cv::Mat &img) {

  ImgSegmenter segmenter(img);

  median_kernel = 3;
  normal_radius = 3;

  // Median Filter for Noise Reduction
  cv::medianBlur(image, median_img, median_kernel);

  // Estimation of Surface Normals
  estimateNormals(median_img, normal_radius, normals);

  // Prints Normal Vector at every pixel and Map xyz to RGB
  //*
  Rect crop(normal_radius, normal_radius, norm_color_img.cols - 2 * normal_radius,
            norm_color_img.rows - 2 * normal_radius); // Crop image according to normal_radius

  norm_color_img = norm_color_img(crop);
  norm_img = norm_img(crop);

  print_normals(norm_img, norm_color_img, normals, normal_radius, kernel_normals);


  //*/

  // Detection of Surface Normal Edges
  //*

  edge_radius = 4;
  norm_edge_img = norm_edge_img(crop);
  detect_normal_edges(norm_edge_img, norm_bin_edge_img, normals, normal_radius,
                      edge_radius);
  crop = Rect(edge_radius, edge_radius,
              norm_edge_img.cols - 2 * edge_radius,
              norm_edge_img.rows - 2 * edge_radius);
  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_bin_edge_img(crop);

  //*/

  //*
  begin = clock();
  cout << "Region Growing begin \n";
  
  regions = 0;
  cvtColor(norm_bin_edge_img, colored, CV_GRAY2RGB);
  
  color_regions(median_img, colored, regions);

  write_tofile(median_img, colored);

  cout << "Regions found: " << regions << "\n";
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Region Growing end \n";
  cout << "Elapsed time: " << elapsed_secs << "\n";
  //*/

  return 0;
}