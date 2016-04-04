#include "segmentation.hpp"

int prepare_segmentation(int argc, char **argv) {

  if (argc != 2) {
    cout << "Usage: DisplayImage.out <Image_Path>\n";
    return -1;
  }

  image = imread(argv[1], CV_8UC1);

  if (!image.data) {
    cout << "No image data \n";
    return -1;
  }

  kernel_size = 3;
  // Median Filter for Noise Reduction
  medianBlur(image, median_img, kernel_size);

  // Estimation of Surface Normals
  //*
  begin = clock();
  cout << "Estimation of Surface Normals begin \n";

  radius = 3;
  normals.reserve((median_img.rows - 2 * radius - 1) *
                  (median_img.cols - 2 * radius - 1));
  estimate_normals(median_img, radius, normals);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Estimation of Surface Normals end \nElapsed time: " << elapsed_secs
       << "\n";
  //*/

  // Prints Normal Vector at every pixel and Map xyz to RGB
  //*
  begin = clock();
  cout << "Printing of Surface Normals begin \n";

  norm_img = median_img.clone();
  norm_color_img = median_img.clone();

  kernel_normals = 10;

  cvtColor(image, norm_color_img,
           CV_GRAY2RGB); // Make norm_color_img 3 channels
  Rect crop(radius, radius, norm_color_img.cols - 2 * radius,
            norm_color_img.rows - 2 * radius); // Crop image according to radius

  norm_color_img = norm_color_img(crop);
  norm_img = norm_img(crop);

  print_normals(norm_img, norm_color_img, normals, radius, kernel_normals);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Printing of Surface Normals end \nElapsed time: " << elapsed_secs
       << "\n";
  //*/

  // Detection of Surface Normal Edges
  //*
  begin = clock();
  cout << "Detection of Surface Normal Edges begin \n";

  norm_edge_img = median_img.clone();
  kernel_normedge = 4;
  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_edge_img.clone();

  detect_normal_edges(norm_edge_img, norm_bin_edge_img, normals, radius,
                      kernel_normedge);

  crop = Rect(kernel_normedge, kernel_normedge,
              norm_edge_img.cols - 2 * kernel_normedge,
              norm_edge_img.rows - 2 * kernel_normedge);
  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_bin_edge_img(crop);

  medianBlur(norm_bin_edge_img, norm_bin_edge_img, 5);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Detection of Surface Normal Edges end \nElapsed time: "
       << elapsed_secs << "\n";
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

  // Shows Images
  namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
  imshow("Original Image", image);

  // namedWindow("Median Image", CV_WINDOW_AUTOSIZE );
  // imshow("Median Image", median_img);

  // namedWindow("Surface Normals Image", CV_WINDOW_AUTOSIZE );
  // imshow("Surface Normals Image", norm_img);

  // namedWindow("Surface Normals Image Mapped to RGB", CV_WINDOW_AUTOSIZE );
  // imshow("Surface Normals Image Mapped to RGB", norm_color_img);

  // namedWindow("Surface Normal Edges Image", CV_WINDOW_AUTOSIZE );
  // imshow("Surface Normal Edges Image", norm_edge_img);

  namedWindow("Surface Normal Binary Edges Image", CV_WINDOW_AUTOSIZE);
  imshow("Surface Normal Binary Edges Image", norm_bin_edge_img);

  namedWindow("Colored Regions Image", CV_WINDOW_AUTOSIZE);
  imshow("Colored Regions Image", colored);

  waitKey(0);
}

void estimate_normals(const Mat &img, const int radius,
                      std::vector<Point3f> &normals) {
  for (int i = radius; i < img.rows - radius; ++i) {
    for (int j = radius; j < img.cols - radius; ++j) {
      // Points a, b, c in the neighborhood of pixel (i, j)
      Point3f a(i + radius, j - radius,
                (float)img.at<uchar>(i + radius, j - radius));
      Point3f b(i + radius, j + radius,
                (float)img.at<uchar>(i + radius, j + radius));
      Point3f c(i - radius, j, (float)img.at<uchar>(i - radius, j));
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
}

void print_normals(Mat &arrowed_dst, Mat &color_dst,
                   const std::vector<Point3f> normals, const int radius,
                   const int kernel) {
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
}

void detect_normal_edges(Mat &dst, Mat &dst_bin,
                         const std::vector<Point3f> norm, const int radius,
                         const int kernel_normedge) {
  for (int i = 0; i < dst.rows; ++i) {
    for (int j = 0; j < dst.cols; ++j) {
      if (i >= kernel_normedge && j >= kernel_normedge &&
          i < dst.rows - kernel_normedge && j < dst.cols - kernel_normedge) {
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

        for (int count = 1; count <= kernel_normedge; ++count) {
          // North Direction
          Point3f n = normals[(i - count) * dst.cols + j];
          costhetan += n0.dot(n) / kernel_normedge;
          // South Direction
          Point3f s = normals[(i + count) * dst.cols + j];
          costhetas += n0.dot(s) / kernel_normedge;
          // West Direction
          Point3f w = normals[i * dst.cols + (j - count)];
          costhetaw += n0.dot(w) / kernel_normedge;
          // East Direction
          Point3f e = normals[i * dst.cols + (j + count)];
          costhetae += n0.dot(e) / kernel_normedge;
          // North West Direction
          Point3f nw = normals[(i - count) * dst.cols + (j - count)];
          costhetanw += n0.dot(nw) / kernel_normedge;
          // North East Direction
          Point3f ne = normals[(i - count) * dst.cols + (j + count)];
          costhetane += n0.dot(ne) / kernel_normedge;
          // South West Direction
          Point3f sw = normals[(i + count) * dst.cols + (j - count)];
          costhetasw += n0.dot(sw) / kernel_normedge;
          // South East Direction
          Point3f se = normals[(i + count) * dst.cols + (j + count)];
          costhetase += n0.dot(se) / kernel_normedge;
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
}

void color_regions(const Mat &median, Mat &dst, int &numregions) {
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

void write_tofile(const Mat &median, const Mat &colored) { 
  std::ofstream myfile;
  myfile.open ("example.obj");
  
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
