#include "Segmentation.hpp"

ImgSegmenter::ImgSegmenter() {}

ImgSegmenter::ImgSegmenter(const cv::Mat &img)
    : image(img), median_kernel(5), normal_radius(3), normal_step(5),
      edge_radius(4), num_regions(0) {}

ImgSegmenter::ImgSegmenter(cv::Mat &&img)
    : image(img), median_kernel(5), normal_radius(3), normal_step(5),
      edge_radius(4), num_regions(0) { }

void ImgSegmenter::estimateNormals(void) {
  cv::medianBlur(image, median_img, 5);
  begin = clock();
  cout << "Estimation of Surface Normals begin \n";

  normals.reserve((median_img.rows - 2 * normal_radius - 1) *
                  (median_img.cols - 2 * normal_radius - 1));

  for (int i = normal_radius; i < image.rows - normal_radius; ++i) {
    for (int j = normal_radius; j < image.cols - normal_radius; ++j) {
      // Points a, b, c in the neighborhood of pixel (i, j)
      cv::Point3f a, b, c;
      if (image.type() == CV_8UC1) {
        a = cv::Point3f(
            i + normal_radius, j - normal_radius,
            static_cast<float>(image.at<uchar>(i + normal_radius, j - normal_radius)));
        b = cv::Point3f(
            i + normal_radius, j + normal_radius,
            static_cast<float>(image.at<uchar>(i + normal_radius, j + normal_radius)));
        c = cv::Point3f(i - normal_radius, j,
                      static_cast<float>(image.at<uchar>(i - normal_radius, j)));
      } else if (image.type() == CV_16UC1) {
        a = cv::Point3f(
            i + normal_radius, j - normal_radius,
            static_cast<float>(image.at<ushort>(i + normal_radius, j - normal_radius)));
        b = cv::Point3f(
            i + normal_radius, j + normal_radius,
            static_cast<float>(image.at<ushort>(i + normal_radius, j + normal_radius)));
        c = cv::Point3f(i - normal_radius, j,
                      static_cast<float>(image.at<ushort>(i - normal_radius, j)));
      } else {
          std::cout << "Unsupported image type! \n";
          return;
      }

      cv::Point3f n;
      // Check if pixel is not a valid measurment
      if (a.z == 255 || b.z == 255 || c.z == 255) {
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
  cout << "Estimation of Surface Normals end \nElapsed time: " << elapsed_secs
       << "\n";
}

void ImgSegmenter::printNormals(void) {
  begin = clock();
  cout << "Printing of Surface Normals begin \n";

  norm_img = median_img.clone();
  norm_color_img = median_img.clone();

  crop = cv::Rect(normal_radius, normal_radius, norm_color_img.cols - 2 * normal_radius,
          norm_color_img.rows - 2 * normal_radius); // Crop image according to radius

  norm_color_img = norm_color_img(crop);
  norm_img = norm_img(crop);

  cvtColor(median_img, norm_color_img,
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

void ImgSegmenter::detectNormalEdges(void) {
  begin = clock();
  cout << "Detection of Surface Normal Edges begin \n";

  norm_edge_img = median_img(crop);
  norm_bin_edge_img = norm_edge_img.clone();

  for (int i = 0; i < norm_edge_img.rows; ++i) {
    for (int j = 0; j < norm_edge_img.cols; ++j) {
      if (i >= edge_radius && j >= edge_radius &&
          i < norm_edge_img.rows - edge_radius &&
          j < norm_edge_img.cols - edge_radius) {
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
          // Soutedge_radiush Direction
          cv::Point3f s = normals[(i + count) * norm_edge_img.cols + j];
          costhetas += n0.dot(s) / edge_radius;
          // West Direction
          cv::Point3f w = normals[i * norm_edge_img.cols + (j - count)];
          costhetaw += n0.dot(w) / edge_radius;
          // East Direction
          cv::Point3f e = normals[i * norm_edge_img.cols + (j + count)];
          costhetae += n0.dot(e) / edge_radius;
          // North West Direction
          cv::Point3f nw =
              normals[(i - count) * norm_edge_img.cols + (j - count)];
          costhetanw += n0.dot(nw) / edge_radius;
          // North East Direction
          cv::Point3f ne =
              normals[(i - count) * norm_edge_img.cols + (j + count)];
          costhetane += n0.dot(ne) / edge_radius;
          // South West Direction
          cv::Point3f sw =
              normals[(i + count) * norm_edge_img.cols + (j - count)];
          costhetasw += n0.dot(sw) / edge_radius;
          // South East Direction
          cv::Point3f se =
              normals[(i + count) * norm_edge_img.cols + (j + count)];
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

        if (i == 150 && j == 180) {
            imwrite("edges2.png", norm_edge_img);
        }
      }
    }
  }

  crop = cv::Rect(edge_radius, edge_radius,
                  norm_edge_img.cols - 2 * edge_radius,
                  norm_edge_img.rows - 2 * edge_radius);

  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_bin_edge_img(crop);

  cv::medianBlur(norm_bin_edge_img, norm_bin_edge_img, 5);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Detection of Surface Normal Edges end \nElapsed time: "
       << elapsed_secs << "\n";
}

void ImgSegmenter::colorRegions(void) {
  begin = clock();
  cout << "Region Growing begin \n";
  colored_img = cv::Mat(cv::Size(norm_bin_edge_img.cols, norm_bin_edge_img.rows), CV_8UC1);

//  norm_bin_edge_img.convertTo(colored_img, CV_8UC1, 255.0/2048.0);
  //norm_bin_edge_img.convertTo(norm_bin_edge_img, CV_8UC1, 255.0/2048.0);
  cvtColor(colored_img, colored_img, CV_GRAY2RGB);

  for (int i = 0; i < colored_img.rows; ++i) {
    for (int j = 0; j < colored_img.cols; ++j) {
      cv::Point seed;
      if (colored_img.at<cv::Vec3b>(i, j) == cv::Vec3b(255, 255, 255)) {
        ++num_regions;
        seed = cv::Point(j, i);
        cv::floodFill(colored_img, seed, getNextColor(num_regions));
      }
    }
  }

  int ke = 7;
  for (int r = 0; r < colored_img.rows; ++r) {
    for (int c = 0; c < colored_img.cols; ++c) {
      if (colored_img.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0)) {
        cv::Point min_distp = cv::Point(0, 0);
        float min_dist = 3;
        for (int i = -ke / 2; i <= ke / 2; ++i) {
          for (int j = -ke / 2; j < ke / 2; ++j) {
            int image_r =
                std::min(std::max(r + i, 0), (int)(colored_img.rows - 1));
            int image_c =
                std::min(std::max(c + j, 0), (int)(colored_img.cols - 1));
            if (colored_img.at<cv::Vec3b>(image_r, image_c) !=
                cv::Vec3b()) {
              float depth_dif =
                  abs(static_cast<float>(median_img.at<uchar>(image_r, image_c) -
                      static_cast<float>(median_img.at<uchar>(r, c))));
              float euc_dist = sqrt(pow(i, 2) + pow(j, 2) + pow(depth_dif, 2));
              if (euc_dist < min_dist) {
                min_dist = euc_dist;
                min_distp = cv::Point(image_c, image_r);
              }
            }
          }
        }
        if (min_distp != cv::Point(0, 0)) {
          colored_img.at<cv::Vec3b>(r, c) =
              colored_img.at<cv::Vec3b>(min_distp);
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

void ImgSegmenter::writetoFile(std::string filename) {
  std::ofstream myfile;
  myfile.open(filename + ".obj");

  for (int i = 0; i < colored_img.rows; ++i) {
    for (int j = 0; j < colored_img.cols; ++j) {
      if ((i % 2 == 0) && (j % 2 == 0)) {
        // cout << "I = " << i << "J = " << j << "\n";
        myfile << "v " << ((colored_img.cols / 2.0) - j) / colored_img.cols
               << " " << ((colored_img.rows / 2.0) - i) / colored_img.rows
               << " " << (int)median_img.at<uchar>(i, j) / 255. << "\n";
        myfile << "c " << colored_img.at<cv::Vec3b>(i, j)[0] / 255. << " "
               << colored_img.at<cv::Vec3b>(i, j)[1] / 255. << " "
               << colored_img.at<cv::Vec3b>(i, j)[2] / 255. << "\n";
      }
    }
  }
  cout << colored_img.rows << " " << (int)colored_img.cols / 2 << "\n";
  myfile.close();
}

void ImgSegmenter::writetoMesh(Mesh &m, int step) {
    std::cout << "Starting writing mesh. \n";
    for (int i = 0; i < colored_img.rows; i += step) {
      for (int j = 0; j < colored_img.cols; j += step) {
//          int depthValue = static_cast<int>(image.at<uchar>(i, j));
//          float depth;
//          if (depthValue < 254) {
//            depth = static_cast<float>(1.0 / ((static_cast<double>(depthValue) * (-0.0030711016)) + 3.3309495161));
//          } else {
//            continue;
//          }
//          double fx_d = 1.0 / 5.9421434211923247e+02;
//          double fy_d = 1.0 / 5.9104053696870778e+02;
//          double cx_d = 3.3930780975300314e+02;
//          double cy_d = 2.4273913761751615e+02;
//          float x = static_cast<float>((j - cy_d) * depth * fy_d);
//          float y = -static_cast<float>((i - cx_d) * depth * fx_d);
//          float z = static_cast<float>(depth);
//          Vertex v(x, y, z);

        int index = i * (colored_img.cols + 2) + j + 2;
        Vertex v(- ((image.cols / 2.0) - j) / image.cols,
                 ((image.rows / 2.0) - i) / image.rows,
                   image.at<uchar>(i, j) / 255.);
        Vertex n(normals[index].x, normals[index].y, normals[index].z);
        m.vertices.push_back(v);
        //m.normals.push_back(n);
        if (i < colored_img.rows - 1 && j < colored_img.cols - 1) {
            Triangle t(i * colored_img.cols + j,
                       i * colored_img.cols + j + 1,
                       (i + 1) * colored_img.cols + j);
            m.triangles.push_back(t);
        }
        if (i > 0 && j < colored_img.cols - 1) {
            Triangle t(i * colored_img.cols + j,
                       i * colored_img.cols + j + 1,
                       (i - 1) * colored_img.cols + j);
            m.triangles.push_back(t);
        }
        if (!colored_img.empty()) {
          cv::Vec3b color(cv::Vec3b(colored_img.at<cv::Vec3b>(i, j)));
          m.colors.push_back(color);
        }
      }
    }
    m.printInfo();
    std::cout << "Ended writing mesh. \n";
}

void ImgSegmenter::writetoMesh(Mesh &m, int step, cv::Mat im, cv::Vec3b mask) {
  for (int i = 0; i < im.rows; ++i) {
    for (int j = 0; j < im.cols; ++j) {
      if ((i % step == 0) && (j % step == 0) && im.at<cv::Vec3b>(i, j) == mask) {
        Vertex v(-((im.cols / 2.0) - j) / im.cols,
                 ((im.rows / 2.0) - i) / im.rows,
                  image.at<uchar>(i, j) / 255.);
        m.vertices.push_back(v);
        cv::Vec3b color(cv::Vec3b(im.at<cv::Vec3b>(i, j)));
        m.colors.push_back(color);
      }
    }
  }
}

cv::Scalar ImgSegmenter::getNextColor(int seed) {
    cv::Mat hsv(1, 1, CV_8UC3);
    cv::Mat rgb(1, 1, CV_8UC3);
    int hue = (seed * 5) % 255;
    int sat = 255 - (seed % 255) * 5;

    cv::cvtColor(hsv, hsv, CV_RGB2HSV);
    hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(hue, sat, 255);
    cv::cvtColor(hsv, rgb, CV_HSV2RGB);

    return cv::Scalar(rgb.at<cv::Vec3b>(0, 0)[0],
                      rgb.at<cv::Vec3b>(0, 0)[1],
                      rgb.at<cv::Vec3b>(0, 0)[2]);
}

ImgSegmenter::~ImgSegmenter() {}


