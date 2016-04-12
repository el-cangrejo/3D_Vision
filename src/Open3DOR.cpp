#include "Open3DOR.hpp"

#include <armadillo>
#include <boost/filesystem.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <ctime>

Vertex::Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
Vertex::Vertex(void) : x(0.0), y(0.0), z(0.0) {}
Vertex::Vertex(Vertex &&other) = default;
Vertex::Vertex(const Vertex &other) = default;
float Vertex::L2Norm(void) const {
  return sqrt((pow(x, 2) + pow(y, 2) + pow(z, 2)));
}
float Vertex::L2Norm(const Vertex &other) const {
  return sqrt(
      (pow(x - other.x, 2) + pow(y - other.y, 2) + pow(z - other.z, 2)));
}
float Vertex::Dot(const Vertex &other) {
  return (x * other.x + y * other.y + z * other.z);
}
Vertex Vertex::Cross(const Vertex &other) {
  Vertex newvertex;
  newvertex.x = (y * other.z - z * other.y);
  newvertex.y = (z * other.x - x * other.z);
  newvertex.z = (x * other.y - y * other.x);
  return newvertex;
}
Vertex Vertex::Normalize() {
  Vertex newvertex;
  newvertex.x = x / this->L2Norm();
  newvertex.y = y / this->L2Norm();
  newvertex.z = z / this->L2Norm();
  return newvertex;
}
float Vertex::Angle(const Vertex &other) {
  float costheta = (this->Dot(other) / (this->L2Norm() * other.L2Norm()));
  float theta = acos(costheta);
  return theta;
}
Vertex &Vertex::operator=(Vertex &&other) = default;
Vertex &Vertex::operator=(const Vertex &other) = default;
bool Vertex::operator==(const Vertex &other) const {
  return (x == other.x && y == other.y && z == other.z);
}
Vertex Vertex::operator-(const Vertex &other) {
  return Vertex(x - other.x, y - other.y, z - other.z);
}
Vertex Vertex::operator+(const Vertex &other) {
  return Vertex(x + other.x, y + other.y, z + other.z);
}
Vertex Vertex::operator*(const float num) {
  return Vertex(x * num, y * num, z * num);
}
Vertex Vertex::operator/(const float num) {
  if (num != 0)
    return Vertex(x / num, y / num, z / num);
  return Vertex(0, 0, 0);
}
Vertex::~Vertex() {}

Edge::Edge(int v1, int v2) : v1(v1), v2(v2) {}
Edge::Edge(void) : v1(0), v2(0) {}
Edge::Edge(Edge &&e) = default;
Edge::Edge(const Edge &e) = default;
Edge &Edge::operator=(Edge &&e) = default;
Edge &Edge::operator=(const Edge &e) = default;
bool Edge::operator==(const Edge &other) const {
  return ((v1 == other.v1 && v2 == other.v2) ||
          (v2 == other.v1 && v1 == other.v2));
}
Edge::~Edge() {}

Triangle::Triangle(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {}
Triangle::Triangle(void) : v1(0), v2(0), v3(0) {}
Triangle::Triangle(Triangle &&t) = default;
Triangle::Triangle(const Triangle &t) = default;
bool Triangle::areNeighbors(const Triangle &other) {
  Edge e1(v1, v2);
  Edge e2(v2, v3);
  Edge e3(v3, v1);

  Edge eo1(other.v1, other.v2);
  Edge eo2(other.v2, other.v3);
  Edge eo3(other.v3, other.v1);

  if (e1 == eo1 || e1 == eo2 || e1 == eo3)
    return true;
  if (e2 == eo1 || e2 == eo2 || e2 == eo3)
    return true;
  if (e3 == eo1 || e3 == eo2 || e3 == eo3)
    return true;

  return false;
}
float Triangle::Area(std::vector<Vertex> &v) {
  Vertex diff1 = v[v2] - v[v1];
  Vertex diff2 = v[v3] - v[v1];
  Vertex n = diff1.Cross(diff2);
  float area = n.L2Norm() / 2.;
  return area;
}
Triangle &Triangle::operator=(Triangle &&t) = default;
Triangle &Triangle::operator=(const Triangle &t) = default;
Triangle::~Triangle() {}

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
    computeNormals_PCA();
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
void Mesh::computeNormals_PCA(void) {}
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
  for (auto &v : vertices) {
    v = v - centroid;
  }
}
bool Mesh::empty(void) { return vertices.empty(); }
void Mesh::clear(void) {
  vertices.clear();
  vertices.shrink_to_fit();
  triangles.clear();
  triangles.shrink_to_fit();
  normals.clear();
  normals.shrink_to_fit();
  fpfhist.clear();
  fpfhist.shrink_to_fit();
  voxel_grid.clear();
  voxel_grid.shrink_to_fit();
}
Mesh Mesh::gridFilter(void) {
  std::cout << "Start grid filtering.." << std::endl;

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

  Mesh fil_mesh;

  Vertex max(0., 0., 0.);
  Vertex min(1., 1., 1.);

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
          voxel_centroid_normal =
              voxel_centroid_normal + normals[pointIdxRadiusSearch[i]];
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

void read_mesh(const std::string filepath, Mesh &mesh) {
  if (filepath.substr(filepath.size() - 3, 3) != "obj") {
    std::cout << "Wrong file format!\n";
    return;
  }
  std::ifstream objfile;
  std::string line;
  int type(0);
  float maxdist(0.0);

  std::vector<Vertex> std_normals;

  mesh.vertices.clear();
  mesh.vertices.shrink_to_fit();
  mesh.triangles.clear();
  mesh.triangles.shrink_to_fit();
  mesh.normals.clear();
  mesh.normals.shrink_to_fit();

  // Reads .obj File
  objfile.open(filepath);
  if (objfile.is_open()) {
    std::cout << "Started reading " << filepath << "\n";
    while (getline(objfile, line)) {
      std::istringstream in(line);
      std::string element;
      in >> element;

      if (element == "v") {
        float x, y, z;
        in >> x >> y >> z;
        mesh.vertices.push_back(Vertex(x, y, z));

        mesh.centroid.x += x;
        mesh.centroid.y += y;
        mesh.centroid.z += z;

        float dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        if (dist > maxdist)
          maxdist = dist;
      } else if (element == "vn" || element == "n") {
        float nx, ny, nz;
        in >> nx >> ny >> nz;
        Vertex n = Vertex(nx, ny, nz);
        std_normals.push_back(n.Normalize());
      } else if (element == "f") {
        if (type == 0 && !std_normals.empty())
          mesh.normals.resize(mesh.vertices.size());
        if (type == 0)
          type = find_type(line);
        int v1, v2, v3;
        int vn1, vn2, vn3;
        int vt1, vt2, vt3;

        switch (type) {
        case 1: {
          in >> v1 >> v2 >> v3;
          break;
        }
        case 2: {
          char c;
          in >> v1 >> c >> vt1;
          in >> v2 >> c >> vt2;
          in >> v3 >> c >> vt3;
          break;
        }
        case 3: {
          char c;
          in >> v1 >> c >> c >> vn1;
          in >> v2 >> c >> c >> vn2;
          in >> v3 >> c >> c >> vn3;
          break;
        }
        case 4: {
          char c;
          in >> v1 >> c >> vt1 >> c >> vn1;
          in >> v2 >> c >> vt2 >> c >> vn2;
          in >> v3 >> c >> vt3 >> c >> vn3;
          break;
        }
        default: {
          std::cout << "No such type! \n";
          exit(0);
          break;
        }
        }

        if ((type == 3 || type == 4) && !std_normals.empty()) {
          mesh.normals[v1 - 1] = std_normals[vn1 - 1];
          mesh.normals[v2 - 1] = std_normals[vn2 - 1];
          mesh.normals[v3 - 1] = std_normals[vn3 - 1];
        } else if ((type == 1 || type == 2) && !std_normals.empty()) {
          if (mesh.normals.size() != std_normals.size())
            std::cout << "fuck\n";
          mesh.normals[v1 - 1] = std_normals[v1 - 1];
          mesh.normals[v2 - 1] = std_normals[v2 - 1];
          mesh.normals[v3 - 1] = std_normals[v3 - 1];
        }
        mesh.triangles.push_back(Triangle(v1 - 1, v2 - 1, v3 - 1));
      }
    }
    objfile.close();
  } else {
    std::cout << "Unable to open file! \n";
  }

  mesh.centroid.x /= mesh.vertices.size();
  mesh.centroid.y /= mesh.vertices.size();
  mesh.centroid.z /= mesh.vertices.size();

  mesh.printInfo();
}

void preprocess_mesh(Mesh &mesh) {
  mesh.movetoCenter();
  mesh.fittoUnitSphere();
  mesh.computeNormals();
  mesh.computeFPFH();
  // std::vector<Edge> v;
  // Copies Unique Edges
  // std::copy_if(v.begin(), v.end(), std::back_inserter(mesh.edges),
  //   [](Edge &e) { auto result = std::find(mesh.edges.begin(),
  //   mesh.edges.end(), e);
  //                 if (result == mesh.edges.end()) return true;
  //                 return false;});

  // Finds the Dual Vertices for every Vertex
  // mesh.computeDualVertices();

  // Finds the Dual Edges for every Dual Vertex
  // mesh.computeDualEdges();

  // mesh.findNeighbors();
}

float local_distance(const Mesh &query_mesh, const Mesh &target_mesh) {
  clock_t begin, end;
  double elapsed_secs;
  begin = clock();
  std::cout << "Calculating local distance\n";

  float dmw2(0.0);
  int count(0);

  // std::cout << "Query mesh size : " << query_mesh.fpfhist.size() << "\n";
  // std::cout << "Target mesh size : " << target_mesh.fpfhist.size() << "\n";

  for (size_t idx_q = 0; idx_q < query_mesh.fpfhist.size(); idx_q += 2) {
    int k(5);
    std::vector<float> min_dists(k, 10000000.0);

    for (size_t idx_t = 0; idx_t < target_mesh.fpfhist.size(); idx_t += 4) {
      // std::cout << "Query " << query_mesh.fpfhist[idx_q].size() << "\n";
      // std::cout << "Target " << target_mesh.fpfhist[idx_t].size() << "\n";
      float dist =
          dist_L1(query_mesh.fpfhist[idx_q], target_mesh.fpfhist[idx_t]);

      // std::cout << "Distance is : " << dist << "\n";

      for (int i = 0; i < k; ++i)
        if (dist < min_dists[i])
          min_dists[i] = dist;
    }
    ++count;
    for (int i = 0; i < k; ++i) {
      // std::cout << "Distance to add = " << (1. / k) * (1 - (float)i / k) *
      // min_dists[i] << "\n";
      dmw2 += (1. / k) * (1 - (float)i / k) * min_dists[i];
    }
  }

  // std::cout << "Local distance before averaging = " << dmw2 << "\n";
  // std::cout << "count = " << count << "\n";
  dmw2 /= count;

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Calculating local distance end : " << dmw2
            << "\nelapsed time: " << elapsed_secs << "\n";

  return dmw2;
}

float dist_L1(const std::vector<float> &hist1,
              const std::vector<float> &hist2) {

  // std::cout << "hist1 size : " << hist1.size() << "\n";
  // std::cout << "hist2 size : " << hist2.size() << "\n";

  if (hist1.size() != hist2.size()) {
    std::cout << "Wrong histogram sizes!\n";
    exit(0);
  }

  float dist(0.0);

  for (size_t i = 0; i < hist1.size(); ++i) {
    dist += fabs(hist1[i] - hist2[i]);
    // std::cout << "Hist1 : " << hist1[i] << "\n";
    // std::cout << "Hist2 : " << hist2[i] << "\n";
  }
  return dist;
}

int find_type(const std::string line) {
  int count(0);

  for (size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '/')
      ++count;
  }
  if (count == 0)
    return 1;
  if (count == 3)
    return 2;
  for (size_t i = 0; i < line.size() - 1; ++i)
    if ((line[i] == '/') && (line[i + 1]) == '/')
      return 3;

  return 4;
}

void save_descriptors(const std::string filepath, const Mesh &mesh) {
  std::ofstream descriptor_file(filepath);

  std::cout << "Saving descriptor of target\n";
  if (descriptor_file.is_open()) {
    for (const auto &hist : mesh.fpfhist) {
      for (const auto &value : hist) {
        descriptor_file << value << " ";
      }
      descriptor_file << "\n";
    }
    descriptor_file.close();
  } else {
    std::cout << "Could not open file to save descriptors!\n";
  }
}

std::vector<std::vector<float>> load_descriptors(const std::string filepath) {
  std::vector<std::vector<float>> descr;
  std::ifstream descriptor_file(filepath);
  std::string line;
  std::cout << "Loading file " << filepath << "\n";
  if (descriptor_file.is_open()) {
    while (getline(descriptor_file, line)) {
      std::istringstream in(line);
      std::vector<float> hist;
      int hist_size = 66;
      hist.resize(hist_size);
      for (int i = 0; i < hist_size; ++i) {
        in >> hist[i];
      }
      descr.push_back(hist);
    }
  } else {
    std::cout << "Could not open file to load descriptors\n";
  }
  return descr;
}

void preprocess_database(std::string db_path) {
  boost::filesystem::path p(db_path);

  if (!boost::filesystem::is_directory(p)) {
    std::cout << "Failed to open DB.. \n";
    exit(1);
  }

  int count(0);

  for (boost::filesystem::directory_iterator dir(p);
       dir != boost::filesystem::directory_iterator(); dir++) {
    if (!boost::filesystem::is_regular_file(*dir))
      continue;

    std::string target_file = (*dir).path().string();

    if (target_file.substr(target_file.size() - 3, 3) != "obj")
      continue;

    Mesh target_mesh;

    read_mesh(target_file, target_mesh);
    preprocess_mesh(target_mesh);
    target_mesh.grid_size = 0.05;
    target_mesh = target_mesh.gridFilter();
    target_file = target_file.substr(0, target_file.size() - 3) + "dtr";
    save_descriptors(target_file, target_mesh);
    ++count;
  }
  std::cout << "Files preprocessed in database : " << count << "\n";
}

void load_database(std::string db_path, std::vector<Mesh> &db_descr,
                   std::vector<std::string> &db_fil) {
  boost::filesystem::path p(db_path);

  if (!boost::filesystem::is_directory(p)) {
    std::cout << "Failed to open DB.. \n";
    exit(1);
  }

  int count(0);

  for (boost::filesystem::directory_iterator dir(p);
       dir != boost::filesystem::directory_iterator(); dir++) {
    if (!boost::filesystem::is_regular_file(*dir))
      continue;

    std::string target_file = (*dir).path().string();

    if (target_file.substr(target_file.size() - 3, 3) != "dtr")
      continue;

    Mesh target_mesh;
    target_mesh.fpfhist = std::move(load_descriptors(target_file));
    std::string target_obj =
        target_file.substr(0, target_file.size() - 3) + "obj";
    db_fil.push_back(target_obj);
    // std::cout << "Hist size : " << target_mesh.fpfhist[0].size() << "\n";
    db_descr.push_back(target_mesh);
    ++count;
  }
  std::cout << "Files preprocessed in database : " << count << "\n";
}
