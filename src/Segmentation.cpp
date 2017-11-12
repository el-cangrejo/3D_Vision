#include "Segmentation.hpp"

ImgSegmenter::ImgSegmenter() {}

ImgSegmenter::ImgSegmenter(const cv::Mat &img)
    : image(img), median_kernel(3), normal_radius(3), normal_step(10),
      edge_radius(5), num_regions(0), kernel_radius(0) {
    if (image.type() == CV_16UC1) {
      image.convertTo(orig_img, CV_8UC1, 255.0 /  2048.0);
    } else {
      image.copyTo(orig_img);
    }

    median_img = orig_img.clone();
    norm_img = orig_img.clone();
    norm_img.convertTo(norm_color_img, CV_8UC3);
    norm_edge_img = orig_img.clone();
    norm_bin_edge_img = orig_img.clone();
}

ImgSegmenter::ImgSegmenter(cv::Mat &&img)
    : image(img), median_kernel(3), normal_radius(3), normal_step(10),
      edge_radius(5), num_regions(0), kernel_radius(0) {
    if (image.type() == CV_16UC1) {
      image.convertTo(orig_img, CV_8UC1, 255.0 /  2048.0);
    } else {
      image.copyTo(orig_img);
    }

    median_img = orig_img.clone();
    norm_img = orig_img.clone();
    norm_img.convertTo(norm_color_img, CV_8UC3);
    norm_edge_img = orig_img.clone();
    norm_bin_edge_img = orig_img.clone();
}

ImgSegmenter::ImgSegmenter(cv::Mat &&img, int mednkrnl, int normrds, int edgrds, int kernelrds) : image(img),
    normal_step(10),num_regions(0), kernel_radius(kernelrds) {
    if (image.type() == CV_16UC1) {
      image.convertTo(orig_img, CV_8UC1, 255.0 /  2048.0);
    } else {
      image.copyTo(orig_img);
    }

    median_img = orig_img.clone();
    norm_img = orig_img.clone();
    norm_img.convertTo(norm_color_img, CV_8UC3);
    norm_edge_img = orig_img.clone();
    norm_bin_edge_img = orig_img.clone();

    median_kernel = mednkrnl;
    normal_radius = normrds;
    edge_radius = edgrds;
		cout << "Kernel radius: " << kernel_radius << "\n";
}

void ImgSegmenter::estimateNormals(void) {
  cv::medianBlur(image, median_img, 3);
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
      } else if (a.z > 2000 || b.z > 2000 || c.z > 2000) {
          n = cv::Point3f(0, 0, 0);
      } else if (a.z == 0 || b.z == 0 || c.z == 0) {
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

  crop = cv::Rect(normal_radius, normal_radius, norm_color_img.cols - 2 * normal_radius,
          norm_color_img.rows - 2 * normal_radius); // Crop image according to radius

  norm_color_img = norm_color_img(crop);
  norm_img = norm_img(crop);
  cvtColor(norm_color_img, norm_color_img, CV_GRAY2RGB);

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

  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_bin_edge_img(crop);

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
        if (costheta > 0.92) {
          norm_bin_edge_img.at<uchar>(i, j) = 255;
        } else {
          norm_bin_edge_img.at<uchar>(i, j) = 0;
        }
      }
    }
  }

  crop = cv::Rect(edge_radius, edge_radius,
                  norm_edge_img.cols - 2 * edge_radius,
                  norm_edge_img.rows - 2 * edge_radius);

  norm_edge_img = norm_edge_img(crop);
  norm_bin_edge_img = norm_bin_edge_img(crop);

  //cv::medianBlur(norm_bin_edge_img, norm_bin_edge_img, 5);
  cv::dilate(norm_bin_edge_img, norm_bin_edge_img, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Detection of Surface Normal Edges end \nElapsed time: "
       << elapsed_secs << "\n";
}

void ImgSegmenter::colorRegions(void) {
  begin = clock();
  cout << "Region Growing begin \n";
  cvtColor(norm_bin_edge_img, colored_img, CV_GRAY2RGB);

  for (int i = 0; i < colored_img.rows; ++i) {
    for (int j = 0; j < colored_img.cols; ++j) {
      cv::Point seed;
      if (colored_img.at<cv::Vec3b>(i, j) == cv::Vec3b(255, 255, 255)) {
        ++num_regions;
        seed = cv::Point(j, i);
        cv::Scalar color = getNextColor(num_regions);
        colors.push_back(color);
				cv::floodFill(colored_img, seed, color);
      }
    }
  }

  int ke = kernel_radius;
  for (int r = 0; r < colored_img.rows; ++r) {
    for (int c = 0; c < colored_img.cols; ++c) {
      if (colored_img.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0)) {
        cv::Point min_distp = cv::Point(0, 0);
        float min_dist = 5;
        for (int i = -ke / 2; i <= ke / 2; ++i) {
          for (int j = -ke / 2; j < ke / 2; ++j) {
            int image_r =
                std::min(std::max(r + i, 0), (int)(colored_img.rows - 1));
            int image_c =
                std::min(std::max(c + j, 0), (int)(colored_img.cols - 1));
            if (colored_img.at<cv::Vec3b>(image_r, image_c) !=
                cv::Vec3b()) {
              float depth_dif;
              if (image.type() == CV_16UC1) {
                  depth_dif = fabs(static_cast<float>(image.at<ushort>(image_r, image_c) -
                      static_cast<float>(image.at<ushort>(r, c))));
              } else {
                  depth_dif = fabs(static_cast<float>(image.at<uchar>(image_r, image_c) -
                      static_cast<float>(image.at<uchar>(r, c))));
              }

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
  cout << "Kernel radius: " << kernel_radius << "\n";
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

void ImgSegmenter::writetoMesh_impl(Mesh &m, int step) {
    std::cout << "Starting writing mesh. \n";

		float cx = 257.966;
		float cy = 210.268;

		float fx = 0.00273687;
		float fy = 0.00273687;

    for (int i = 0; i < image.rows; ++i) {
      for (int j = 0; j < image.cols; ++j) {
        Vertex v;

				float depth_val = image.at<ushort>(i, j) / 1000.0f;

				v.x = (i + 0.5 - cx) * fx * depth_val;
				v.y = (j + 0.5 - cy) * fy * depth_val;
				v.z = depth_val;
				m.vertices.push_back(v);
      }
    }
    m.printInfo();
    std::cout << "Ended writing mesh. \n";
}

void ImgSegmenter::writetoMesh(Mesh &m, int step) {
    std::cout << "Starting writing mesh. \n";

		float cx = 257.966;
		float cy = 210.268;

		float fx = 0.00273687;
		float fy = 0.00273687;

    for (int i = 0; i < colored_img.rows; ++i) {
      for (int j = 0; j < colored_img.cols; ++j) {
				Vertex v;

				float depth_val = image.at<ushort>(i, j) / 1000.0f;

				v.y = -(i + 0.5 - cx) * fx * depth_val;
				v.x = (j + 0.5 - cy) * fy * depth_val;
				v.z = -depth_val;
				m.vertices.push_back(v);

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
    std::cout << "Starting writing mesh. \n";
		m.clear();

		float cx = 257.966;
		float cy = 210.268;

		float fx = 0.00273687;
		float fy = 0.00273687;

    for (int i = 0; i < im.rows; ++i) {
      for (int j = 0; j < im.cols; ++j) {
				if (im.at<cv::Vec3b>(i, j) != mask) continue;
				Vertex v;

				float depth_val = image.at<ushort>(i, j) / 1000.0f;

				v.y = -(i + 0.5 - cx) * fx * depth_val;
				v.x = (j + 0.5 - cy) * fy * depth_val;
				v.z = -depth_val;
				//std::cout << "Vertex : " << v.x << " " << v.y << " " << v.z << "\n";
				if (v.x == 0 && v.y == 0 && v.z == 0) continue;
				m.vertices.push_back(v);

				if (!colored_img.empty()) {
					cv::Vec3b color(cv::Vec3b(colored_img.at<cv::Vec3b>(i, j)));
					m.colors.push_back(color);
				}
      }
    }
    m.printInfo();
    std::cout << "Ended writing mesh. \n";
}

void ImgSegmenter::writetoMesh(std::vector<Mesh> &meshes, int step) {
	std::cout << "Starting writing mesh. \n";
	meshes.clear();
	meshes.shrink_to_fit();

	float cx = 257.966;
	float cy = 210.268;

	float fx = 0.00273687;
	float fy = 0.00273687;
	std::cout << "About to write " << colors.size() << " meshes..\n";
	
	Mesh m;
	for (int i = 0; i < colored_img.rows; ++i) {
		for (int j = 0; j < colored_img.cols; ++j) {
			if (colored_img.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0)) continue;
			Vertex v;

			float depth_val = image.at<ushort>(i, j) / 1000.0f;

			v.y = -(i + 0.5 - cx) * fx * depth_val;
			v.x = (j + 0.5 - cy) * fy * depth_val;
			v.z = -depth_val;
			if (v.x == 0 && v.y == 0 && v.z == 0) continue;
			m.vertices.push_back(v);

			if (!colored_img.empty()) {
				cv::Vec3b color(cv::Vec3b(colored_img.at<cv::Vec3b>(i, j)));
				m.colors.push_back(color);
			}
		}
	}
	m.preprocess();
	meshes.push_back(m);
	m.clear();

	for (int k = 0; k < colors.size(); ++k) {
		cv::Vec3b color(colors[k][0], colors[k][1], colors[k][2]);
		int points = 0;
		for (int i = 0 ; i < colored_img.rows; ++i) {
			for (int j = 0; j < colored_img.cols; ++j) {
				if (colored_img.at<cv::Vec3b>(i, j) != color) continue;
				++points;
				Vertex v;
				if (image.type() == CV_16UC1) {
					int depthValue = median_img.at<ushort>(i, j);
					if (depthValue > 2000) continue;

					float depth = depthValue / 1000.0f;

					float y = -(i + 0.5 - cx) * fx * depth;
					float x = (j + 0.5 - cy) * fy * depth;
					float z = -depth;
					v = Vertex(x, y, z);

				} else {
						v = Vertex(-((image.cols / 2.0) - j) / image.cols,
											((image.rows / 2.0) - i) / image.rows,
											 median_img.at<uchar>(i, j) / 255.);
				}
				int index = i * (colored_img.cols + 2) + j + 2;
				Vertex n(normals[index].x, normals[index].y, normals[index].z);
				m.vertices.push_back(v);
				//m.normals.push_back(n);
				if (!colored_img.empty()) {
					//cv::Vec3b color(cv::Vec3b(colored_img.at<cv::Vec3b>(i, j)));
					m.colors.push_back(color);
				}
			}
		}
		if (points > 1000) {
				meshes.push_back(m);
	//			std::cout << "Wrote " << meshes.size() << " mesh\n";
		}
		m.clear();
	}
	
	std::cout << "Wrote " << meshes.size() << "\n";
//	for (int i = 0; i < meshes.size(); ++i) {
//			meshes[i].printInfo();
//			std::cout << "Ended writing mesh. \n";
//	}
}

cv::Scalar ImgSegmenter::getNextColor(int seed) {
    cv::Mat hsv(1, 1, CV_8UC3);
    cv::Mat rgb(1, 1, CV_8UC3);
    int hue = (seed * 5) % 255;
    int sat = 255;
    if (seed * 5 > 255) {
        sat = (seed * 5) - 2 * 255;
    }
    cv::cvtColor(hsv, hsv, CV_RGB2HSV);
    hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(hue, sat, 255);
    cv::cvtColor(hsv, rgb, CV_HSV2RGB);

    return cv::Scalar(rgb.at<cv::Vec3b>(0, 0));
}

ImgSegmenter::~ImgSegmenter() {}


