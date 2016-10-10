#include <Mesh.hpp>

Mesh::Mesh(void) {}

Mesh::~Mesh() {}

void Mesh::computeDualVertices(void) {
  if (triangles.size() == 0 || vertices.size() == 0) {
    std::cout << "Unitialized mesh\n";
  }

  for (const auto &t : triangles) {
    Vertex dv((vertices[t.v1].x + vertices[t.v2].x + vertices[t.v3].x) / 3,
              (vertices[t.v1].y + vertices[t.v2].y + vertices[t.v3].y) / 3,
              (vertices[t.v1].z + vertices[t.v2].z + vertices[t.v3].z) / 3);
    dvertices.push_back(dv);
  }
}

void Mesh::computeDualEdges(void) {
  if (triangles.size() == 0 || vertices.size() == 0 || dvertices.size() == 0) {
    std::cout << "Unitialized mesh\n";
  }

  for (size_t i = 0; i < triangles.size(); ++i) {
    int count = 0;
    for (size_t j = 0; j < triangles.size(); ++j) {
      if (triangles[i].areNeighbors(triangles[j])) {
        Edge de(i, j);
        ++count;
        auto result = std::find(dedges.begin(), dedges.end(), de);
        if (result == dedges.end()) {
          dedges.push_back(de);
        }
      }
      if (count == 3)
        break;
    }
  }
}

void Mesh::findNeighbors(void) {
  std::vector<std::vector<int>> temp_vec(vertices.size());
  for (const auto &e : edges) {
    temp_vec[e.v1].push_back(e.v2);
    temp_vec[e.v2].push_back(e.v1);
  }
  neighbors = temp_vec;
}

std::vector<int> Mesh::findNearestNeighbors(int queryIdx, float radius) {
  std::vector<int> nneighbors;
  Vertex query = vertices[queryIdx];
  for (size_t i = 0; i < vertices.size(); ++i) {
    Vertex test = vertices[i];
    Vertex dif = query - test;
    if (dif.L2Norm() >= radius)
      continue;

    nneighbors.push_back(i);
  }
  return nneighbors;
}

void Mesh::computeNormals(void) {
  if (normals.size() == vertices.size()) {
    std::cout << "Normals already exist!"
              << "\n";
    return;
  }
  if (this->triangles.empty()) {
    //computeNormals_PCA();
    return;
  }

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();
  std::cout << "Calculating normals begin\n";

  std::vector<Vertex> norms(vertices.size());

  for (auto &t : triangles) {
    Vertex pa, pb, pc;
    Vertex diff1, diff2;
    Vertex trinorm;

    pa = vertices[t.v1];
    pb = vertices[t.v2];
    pc = vertices[t.v3];

    diff1 = pb - pa;
    diff2 = pc - pa;
    trinorm = diff1.Cross(diff2);
    trinorm = trinorm.Normalize();
    trinormals.push_back(trinorm);

    float theta1 = (pb - pa).Angle(pc - pa);
    float theta2 = (pa - pb).Angle(pc - pb);
    float theta3 = (pb - pc).Angle(pa - pc);

    norms[t.v1] = norms[t.v1] + trinorm * t.Area(vertices) * theta1;
    norms[t.v2] = norms[t.v2] + trinorm * t.Area(vertices) * theta2;
    norms[t.v3] = norms[t.v3] + trinorm * t.Area(vertices) * theta3;
  }

  for (auto &n : norms) {
    Vertex v = n.Normalize();
    normals.push_back(v);
  }

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Calculating normals end : elapsed time: " << elapsed_secs
            << "\n";
}

void Mesh::computeNormals_PCA(void) {
    clock_t begin, end;
    double elapsed_secs;
    begin = clock();
    std::cout << "Calculating normals PCA begin\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->points.resize(vertices.size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      cloud->points[i] = pcl::PointXYZ(vertices[i].x,
                                       vertices[i].y,
                                       vertices[i].z);
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setRadiusSearch (0.03);
    ne.setViewPoint(0, 0, 3.5);
    ne.compute (*cloud_normals);

    normals.resize(cloud_normals->points.size());
    for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
      normals[i] = Vertex(cloud_normals->points[0].normal_x,
                          cloud_normals->points[0].normal_y,
                          cloud_normals->points[0].normal_z);
//      normals[i].x = cloud_normals->points[0].x;
//      normals[i].y = cloud_normals->points[0].y;
//      normals[i].z = cloud_normals->points[0].z;
    }

    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Calculating normals PCA end : elapsed time: " << elapsed_secs
              << "\n";
}

void Mesh::computeFPFH(void) {

  clock_t begin, end;
  double elapsed_secs;

  begin = clock();
  std::cout << "Calculating FPFH begin\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr nors(new pcl::PointCloud<pcl::Normal>());
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_in;
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_out1;
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_out2;

  cloud->points.resize(vertices.size());
  nors->points.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    cloud->points[i].x = vertices[i].x;
    cloud->points[i].y = vertices[i].y;
    cloud->points[i].z = vertices[i].z;

    nors->points[i].normal[0] = normals[i].x;
    nors->points[i].normal[1] = normals[i].y;
    nors->points[i].normal[2] = normals[i].z;
  }

  fpfh_in.setInputCloud(cloud);
  fpfh_in.setInputNormals(nors);

  fpfh_out1.setInputCloud(cloud);
  fpfh_out1.setInputNormals(nors);

  fpfh_out2.setInputCloud(cloud);
  fpfh_out2.setInputNormals(nors);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3(
      new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh_in.setSearchMethod(tree1);
  fpfh_out1.setSearchMethod(tree2);
  fpfh_out2.setSearchMethod(tree3);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhist_in(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_in.setRadiusSearch(0.05);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhist_out1(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_out1.setRadiusSearch(0.15);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhist_out2(
      new pcl::PointCloud<pcl::FPFHSignature33>());

  fpfh_out2.setRadiusSearch(0.25);

  fpfh_in.compute(*fpfhist_in);
  fpfh_out1.compute(*fpfhist_out1);
  fpfh_out2.compute(*fpfhist_out2);

  fpfhist.resize(fpfhist_in->points.size());
  for (size_t i = 0; i < fpfhist_in->points.size(); ++i) {
    fpfhist[i].resize(66);
    for (int j = 0; j < 33; ++j) {
      fpfhist[i][j] = fpfhist_in->points[i].histogram[j];
      fpfhist[i][j + 33] = fpfhist_out2->points[i].histogram[j] -
                           fpfhist_out1->points[i].histogram[j];
    }
  }

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Calculating FPFH end : elapsed time: " << elapsed_secs << "\n";
  // std::cout << "Fpfhs size = " << fpfhs->points.size() << "\n";
}

void Mesh::fittoUnitSphere(void) {
  float max_dist(0.0);
  for (const auto &v : vertices)
    if (max_dist < v.L2Norm())
      max_dist = v.L2Norm();

  for (auto &v : vertices) {
    v.x /= max_dist;
    v.y /= max_dist;
    v.z /= max_dist;
  }
}

void Mesh::movetoCenter(void) {
  if (centroid == Vertex(0, 0, 0)) {
    for (auto &v : vertices) {
      centroid = centroid + v;
    }
  }
  centroid = centroid / vertices.size();
  for (auto &v : vertices) {
    v = v - centroid;
  }
}

bool Mesh::empty(void) { return vertices.empty(); }

void Mesh::clear(void) {
  vertices.clear();
  vertices.shrink_to_fit();
  colors.clear();
  colors.shrink_to_fit();
  triangles.clear();
  triangles.shrink_to_fit();
  normals.clear();
  normals.shrink_to_fit();
  fpfhist.clear();
  fpfhist.shrink_to_fit();
  voxel_grid.clear();
  voxel_grid.shrink_to_fit();
  centroid = Vertex();
  max = Vertex();
  min = Vertex();
}

Mesh Mesh::gridFilter(void) {
  std::cout << "Start grid filtering.." << std::endl;

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

  Mesh fil_mesh;

  max = Vertex(0., 0., 0.);
  min = Vertex(1., 1., 1.);

  for (const auto &v : vertices) {
    if (v.x > max.x)
      max.x = v.x;
    if (v.y > max.y)
      max.y = v.y;
    if (v.z > max.z)
      max.z = v.z;
    if (v.x < min.x)
      min.x = v.x;
    if (v.y < min.y)
      min.y = v.y;
    if (v.z < min.z)
      min.z = v.z;
  }

  int dim_x = ceil(fabs(max.x - min.x) / grid_size);
  int dim_y = ceil(fabs(max.y - min.y) / grid_size);
  int dim_z = ceil(fabs(max.z - min.z) / grid_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->points.resize(vertices.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i] =
        pcl::PointXYZ(vertices[i].x, vertices[i].y, vertices[i].z);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  float radius = sqrt(pow(grid_size / 2., 2) + pow(grid_size / 2., 2) +
                      pow(grid_size / 2., 2));

  float displacement_x = min.x + grid_size / 2;
  float displacement_y = min.y + grid_size / 2;
  float displacement_z = min.z + grid_size / 2;

  voxel_grid.erase(voxel_grid.begin(), voxel_grid.end());

  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointXYZ searchPoint;

        searchPoint.x = displacement_x + grid_x * grid_size;
        searchPoint.y = displacement_y + grid_y * grid_size;
        searchPoint.z = displacement_z + grid_z * grid_size;

        // std::cout << "Radius search " << grid_x << " "
        //<< grid_y << " " << grid_z << "\n";
        kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance);

        if (pointIdxRadiusSearch.empty()) {
          voxel_grid.push_back(0);
          continue;
        } else {
          voxel_grid.push_back(1);
        }

        // std::cout << "Points found " << pointIdxRadiusSearch.size() << "\n";
        Vertex voxel_centroid(0., 0., 0.);
        Vertex voxel_centroid_normal(0., 0., 0.);

        std::vector<float> hist;
        if (!fpfhist.empty())
          hist.resize(fpfhist[0].size());

        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
          voxel_centroid = voxel_centroid + vertices[pointIdxRadiusSearch[i]];
          if (!normals.empty()) {
            voxel_centroid_normal = voxel_centroid_normal + normals[pointIdxRadiusSearch[i]];
          }
          if (fpfhist.empty())
            continue;
          for (size_t j = 0; j < fpfhist[0].size(); ++j) {
            hist[j] += fpfhist[pointIdxRadiusSearch[i]][j] /
                       pointIdxRadiusSearch.size();
          }
        }

        voxel_centroid = voxel_centroid / pointIdxRadiusSearch.size();
        fil_mesh.vertices.push_back(voxel_centroid);
        voxel_centroid_normal = voxel_centroid_normal.Normalize();
        fil_mesh.normals.push_back(voxel_centroid_normal);
        fil_mesh.fpfhist.push_back(hist);
      }
    }
  }

  fil_mesh.movetoCenter();
  fil_mesh.fittoUnitSphere();
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Finished filtering..\nElaspsed time : " << elapsed_secs << "\n";
  std::cout << "Filtered mesh size " << fil_mesh.vertices.size() << "\n";
  return fil_mesh;
}

Mesh Mesh::statoutFilter(void) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  cloud->points.resize(vertices.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i] =
        pcl::PointXYZ(vertices[i].x, vertices[i].y, vertices[i].z);
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  Mesh filtered_mesh;

  for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
    filtered_mesh.vertices.push_back(Vertex(cloud_filtered->points[i].x,
                                            cloud_filtered->points[i].y,
                                            cloud_filtered->points[i].z));
  }

  std::cout << "Mesh points before stat out filter : " << this->vertices.size()
            << "\n";
  std::cout << "Mesh points after stat out filter : "
            << filtered_mesh.vertices.size() << "\n";

  return filtered_mesh;
}

void Mesh::printInfo(void) {
  // Prints Information about the mesh
  std::cout << "Object size : \n"
            << this->vertices.size() << " vertices \n"
            << this->triangles.size() << " triangles \n"
            << this->edges.size() << " edges \n"
            << this->normals.size() << " normals \n"
            << this->dvertices.size() << " dvertices \n"
            << this->dedges.size() << " dedges \n"
            << this->trinormals.size() << " trinormals \n";
}
