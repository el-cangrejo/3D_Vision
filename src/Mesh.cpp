#include "Mesh.hpp"
#include <pcl/search/impl/search.hpp>

float l1(const dFPFHSignature66 first, const dFPFHSignature66 second) {
  float distance = 0.0;
  for (int i = 0; i < first.descriptorSize(); ++i) {
    distance += fabs(first.histogram[i] - second.histogram[i]);
  }
  return distance;
}

/** 
 *  Public
 */

// Helper
Mesh::Mesh(void) : centroid(0.0, 0.0, 0.0) {}

Mesh::~Mesh() {}

int Mesh::load(const std::string file_path) {
  // Check if mesh is empty and if not clear it
  if (!this->empty())
    this->clear();

  int success = 0;
  // Check file ending to find file type
  if (file_path.substr(file_path.size() - 3, 3) == "obj") {
    success = loadObj(file_path);
  } else if (file_path.substr(file_path.size() - 3, 3) == "off") {
    success = loadOff(file_path);
  } else {
    std::cout << "Unsupported file format!\nFile not loaded\n";
		return success;
  }

	this->preprocess();
  return success;
}

bool Mesh::empty() { return vertices.empty(); }

void Mesh::clear() {
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

void Mesh::printInfo(void) {
  // Prints Information about the mesh
  std::cout << "\e[1;31;43mObject size : \e[0m \e[1;33m\n"
            << "Vertices: " << this->vertices.size() << "\n"
            << "Triangles: " << this->triangles.size() << "\n"
            << "Normals: " << this->normals.size() << "\n"
            << "Centroid: (" << this->centroid.x << ", " << this->centroid.y << ", "
					 	<< this->centroid.z << ")\e[0m\n";
}

void Mesh::passToPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::Normal>::Ptr &normal_cloud) const {
  if (vertices.empty()) {
    std::cout << "Mesh with no vertices!\n";
    exit(0);
  }
  if (normals.empty()) {
    std::cout << "Mesh with no normals!\n";
    exit(0);
  }

  std::cout << "Start passing to point cloud\n";

  cloud->points.resize(vertices.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i] =
        pcl::PointXYZ(vertices[i].x, vertices[i].y, vertices[i].z);
  }

  normal_cloud->points.resize(normals.size());
  for (size_t i = 0; i < normal_cloud->points.size(); ++i) {
    normal_cloud->points[i] =
        pcl::Normal(normals[i].x, normals[i].y, normals[i].z);
  }
}

// Processing 
void Mesh::preprocess () {
  clock_t begin, end;
  double elapsed_secs;
  begin = clock();
	this->moveToCenter ();
	this->fitToUnitSphere ();
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "Preprocessing took : " << elapsed_secs << "\n";
	std::cout << "Info after preprocessing : " << "\n";
	printInfo ();
}

void Mesh::computeNormals() {
  if (this->normals.size() == this->vertices.size()) {
    std::cout << "\e[1;33mNormals already exist!\e[0m\n";
    return;
  }

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();
  std::cout << "\e[1;34mCalculating normals begin\n";

	if (!this->triangles.empty()) {
		this->computeNormals_Tri();
	} else {
		this->computeNormals_PCA();
	}

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Calculating normals end \nElapsed time: " << elapsed_secs
            << "\e[0m\n";
}

void Mesh::process() {
	std::cout << "Processing mesh!\nCalculating dFPFH signatures!\nCalculating Fisher Vectors\n";
}

float Mesh::distanceTo(const Mesh &other) {
  float globalDistance = 0.0;
  float localDistance = 0.0;

  localDistance = this->localDistanceTo(other);
  globalDistance = this->globalDistanceTo(other);
  std::cout << "Global distance :" << globalDistance << "\n";
  float overallDistance = localDistance + globalDistance;

  return overallDistance;
}

Mesh Mesh::gridFilter() {
  std::cout << "\e[1;35mStart grid filtering" << std::endl;
	std::cout << "Starting with " << this->vertices.size() << " vertices and grid " << grid_size << "\n";
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

  fil_mesh.moveToCenter();
  fil_mesh.fitToUnitSphere();
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Finished filtering\nElaspsed time : " << elapsed_secs << "\n";
  std::cout << "Filtered mesh info\e[0m\n";
	fil_mesh.printInfo();
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

void Mesh::triangulate() {
// Load input file into a PointCloud<T> with an appropriate type
	std::cout << "Start triangulation!\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(vertices.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i] =
        pcl::PointXYZ(vertices[i].x, vertices[i].y, vertices[i].z);
  }
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
	std::cout << "Start triangulation!\n";
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setRadiusSearch (0.08);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

	std::cout << "Start triangulation!\n";
  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

	std::cout << "Start triangulation!\n";
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.15);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (90);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

	std::cout << "Start triangulation!\n";
  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
	//std::cout << triangles << "\n";
  // Finish
  return;
}

/**
 * Private
 */

// Loading functions
int Mesh::loadObj(const std::string path) {
  std::ifstream objfile;
  std::string line;
  int type(0);

  std::vector<Vertex> std_normals;

  // Reads .obj File
  objfile.open(path);
  if (objfile.is_open()) {
    while (getline(objfile, line)) {
      std::istringstream in(line);
      std::string element;
      in >> element;

      if (element == "v") {
        float x, y, z;
        in >> x >> y >> z;
        vertices.push_back(Vertex(x, y, z));

      } else if (element == "vn" || element == "n") {
        float nx, ny, nz;
        in >> nx >> ny >> nz;
        Vertex n = Vertex(nx, ny, nz);
        std_normals.push_back(n.Normalize());
      } else if (element == "f") {
        if (type == 0 && !std_normals.empty())
          normals.resize(vertices.size());
        if (type == 0)
          type = findType(line);
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
          std::cout << "No such type of .obj file!\nFile not loaded!\n";
          return 0;
          break;
        }
        }

        if ((type == 3 || type == 4) && !std_normals.empty()) {
          normals[v1 - 1] = std_normals[vn1 - 1];
          normals[v2 - 1] = std_normals[vn2 - 1];
          normals[v3 - 1] = std_normals[vn3 - 1];
        } else if ((type == 1 || type == 2) && !std_normals.empty()) {
          normals[v1 - 1] = std_normals[v1 - 1];
          normals[v2 - 1] = std_normals[v2 - 1];
          normals[v3 - 1] = std_normals[v3 - 1];
        }
        triangles.push_back(Triangle(v1 - 1, v2 - 1, v3 - 1));
      }
    }
    objfile.close();
    std::cout << "Loaded :" << path << "\n";
  } else {
    std::cout << "Unable to open file! \n";
    return 0;
  }

  printInfo();
  return 1;
}

int Mesh::loadOff(const std::string path) {
  std::ifstream off_file;
  std::string line;

  std::vector<Vertex> std_normals;
  off_file.open(path);

  if (off_file.is_open()) {
    getline(off_file, line);
    getline(off_file, line);

    std::istringstream in(line);
    int v, f, e;
    in >> v >> f >> e;
		std::cout << "Num of Vertices " << v << "\n";

    // Read vertices
    for (int i = 0; i < v; ++i) {
      getline(off_file, line);
      std::istringstream in_v(line);
      float x, y, z;
      in_v >> x >> y >> z;
      vertices.push_back(Vertex(x, y, z));
    }

		getline(off_file, line);
    // Read faces
    for (int i = 0; i < f; ++i) {
      getline(off_file, line);
      std::istringstream in_f(line);
      int x, y, z;
      int n;
      in_f >> n;
      in_f >> x >> y >> z;

      triangles.push_back(Triangle(x, y, z));
			//std::cout << "Triangle " << x << " " << y << " " << z << "\n";
    }
    off_file.close();
    std::cout << "Loaded :" << path << "\n";
  } else {
    std::cout << "Unable to open file! \n";
    return 0;
  }

  this->printInfo();
  return 1;
}

int Mesh::findType(const std::string line) {
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

// Preprocessing
void Mesh::fitToUnitSphere(void) {
  float max_dist = 0.0;

  for (const auto &v : vertices) {
    if (max_dist < v.L2Norm()) {
      max_dist = v.L2Norm();
		}
	}

	for (auto &v : vertices) {
		v = v / max_dist;
	} 
}

void Mesh::moveToCenter(void) {
	for (auto &v : vertices) {
		centroid = centroid + v;
	}
  
	centroid = centroid / vertices.size();
  for (auto &v : vertices) {
    v = v - centroid;
  }
}

// Computing
void Mesh::computeNormals_Tri() {
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
}

void Mesh::computeNormals_PCA() {
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

void Mesh::calculatedFPFHSignatures(float radius, float inner_radius,
                                    float outer_radius) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());

  this->passToPointCloud(cloud, normal);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normal);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  fpfh.setSearchMethod(tree);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_inner(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_outer(
      new pcl::PointCloud<pcl::FPFHSignature33>());

  fpfh.setRadiusSearch(radius);
  fpfh.compute(*fpfhs);

  fpfh.setRadiusSearch(inner_radius);
  fpfh.compute(*fpfhs_inner);

  fpfh.setRadiusSearch(outer_radius);
  fpfh.compute(*fpfhs_outer);

  this->dFPFHSignatures = pcl::PointCloud<dFPFHSignature66>::Ptr(
      new pcl::PointCloud<dFPFHSignature66>);
  this->dFPFHSignatures->points.resize(fpfhs->points.size());
  for (int i = 0; i < this->dFPFHSignatures->points.size(); ++i) {
    this->dFPFHSignatures->points[i].populate(
        fpfhs->points[i], fpfhs_outer->points[i], fpfhs_inner->points[i]);
  }
  std::cout << "Calculated dFPFHSignature : "
            << this->dFPFHSignatures->points.size() << "\n";
  std::cout << "Calculated dFPFHSignature : "
            << this->dFPFHSignatures->points[1].descriptorSize() << "\n";
}

void Mesh::calculateFisherVectors() {
  vl_size dataDim = 66;
  vl_size numClusters = 10;
  vl_size numData = dFPFHSignatures->points.size();

  VlGMM *gmm = vl_gmm_new(VL_TYPE_FLOAT, dataDim, numClusters);
  // float * data = vl_malloc(sizeof(float) * numData * dataDim);
  float *data = new float[numData * dataDim];
  this->fisherVectors = new float[2 * dataDim * numClusters];

  std::cout << "Data size: " << sizeof(float) * numData * dataDim << "\n";

  for (vl_size dataIdx = 0; dataIdx < numData; ++dataIdx) {
    for (vl_size d = 0; d < dataDim; ++d) {
      data[dataIdx * dataDim + d] =
          (float)dFPFHSignatures->points[dataIdx].histogram[d];
    }
  }

  // KMeans initialization parameters
  vl_size maxiterKM = 5;
  vl_size ntrees = 3;
  vl_size maxComp = 20;

  VlKMeans *kmeans = vl_kmeans_new(VL_TYPE_FLOAT, VlDistanceL2);
  vl_kmeans_set_verbosity(kmeans, 0);
  vl_kmeans_set_max_num_iterations(kmeans, maxiterKM);
  vl_kmeans_set_max_num_comparisons(kmeans, maxComp);
  vl_kmeans_set_num_trees(kmeans, ntrees);
  vl_kmeans_set_algorithm(kmeans, VlKMeansLloyd);
  vl_kmeans_set_initialization(kmeans, VlKMeansRandomSelection);
  vl_gmm_set_initialization(gmm, VlGMMKMeans);
  vl_gmm_set_kmeans_init_object(gmm, kmeans);

  // GMM set parameters"
  vl_size maxiter = 5;
  vl_size maxrep = 1;

  vl_gmm_set_max_num_iterations(gmm, maxiter);
  vl_gmm_set_num_repetitions(gmm, maxrep);
  vl_gmm_set_verbosity(gmm, 0);

  vl_gmm_cluster(gmm, data, numData);

  vl_fisher_encode(fisherVectors, VL_TYPE_FLOAT, vl_gmm_get_means(gmm), dataDim,
                   numClusters, vl_gmm_get_covariances(gmm),
                   vl_gmm_get_priors(gmm), data, numData,
                   VL_FISHER_FLAG_IMPROVED);

  delete[] data;
  vl_gmm_delete(gmm);
  vl_kmeans_delete(kmeans);
}

// Distance 
float Mesh::localDistanceTo(const Mesh &other) {
  if (dFPFHSignatures->points.size() != vertices.size()) {
    std::cout << "dFPFH signatures not computed!\n";
    return 0.0;
  }

  float localDistance = 0.0;

  for (int i = 0; i < this->dFPFHSignatures->points.size(); ++i) {
    float minDist = 10000.0;
    for (int j = 0; j < other.dFPFHSignatures->points.size(); ++j) {
      float dist = l1(this->dFPFHSignatures->points[i],
                      other.dFPFHSignatures->points[j]);
      if (minDist > dist) {
        minDist = dist;
      }
    }
    localDistance += minDist;
  }

  localDistance = localDistance / (float)this->dFPFHSignatures->points.size();
  std::cout << "Local similarity: " << localDistance << "\n";
  return localDistance;
}

float Mesh::globalDistanceTo(const Mesh &other) {
  float globalDistance = 0.0;

  vl_size dataDim = 66;
  vl_size numClusters = 10;
  for (int i = 0; i < numClusters * dataDim * 2; ++i) {
    globalDistance += fabs(this->fisherVectors[i] - other.fisherVectors[i]);
  }
  return globalDistance;
}
