#ifndef MESH_HPP
#define MESH_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include "MeshComponents.hpp"

class Mesh {
public:
	// Helper functions
  Mesh();
	int load (const std::string file_path);
  bool empty();
  void clear();
  void printInfo();
  ~Mesh();

	// Processing functions
	void preprocess();
	void computeNormals();
	void process();
	float distanceTo(const Mesh &other);

  Mesh gridFilter();
  Mesh statoutFilter();
	void triangulate();

	// Data members
  Vertex centroid;
  Vertex max, min;
  std::vector<Vertex> vertices;
  std::vector<Triangle> triangles;
  std::vector<Vertex> normals;
  std::vector<Vertex> trinormals;
  std::vector<std::vector<float>> fpfhist;
  std::vector<int> voxel_grid;
  std::vector<cv::Vec3b> colors;
  float grid_size;

private:
	// Loading functions
	int loadObj(const std::string file_path);
	int loadOff(const std::string file_path);
	int findType(const std::string line);

	// Preprocessing functions
  void fitToUnitSphere();
  void moveToCenter();

	// Computing functions
  void computeNormals_Tri();
  void computeNormals_PCA();
  void computeFPFH();

	// Distance functions
	float localDistanceTo(const Mesh &other);
	float globalDistanceTo(const Mesh &other);
};

#endif // MESH_HPP

