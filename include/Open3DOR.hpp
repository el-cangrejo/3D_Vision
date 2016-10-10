#ifndef OPEN3DOR_HPP
#define OPEN3DOR_HPP

#include <iostream>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include "Mesh.hpp"

void read_mesh(const std::string, Mesh &);
int loadObj(const std::string filepath, Mesh &mesh);
int loadOff(const std::string filepath, Mesh &mesh);
int findType(const std::string line);

void preprocess_mesh(Mesh &);
float local_distance(const Mesh &, const Mesh &);
float dist_L1(const std::vector<float> &, const std::vector<float> &);
void save_descriptors(const std::string, const Mesh &);
std::vector<std::vector<float>> load_descriptors(const std::string);
void preprocess_database(const std::string);
void load_database(const std::string, std::vector<Mesh> &,
                   std::vector<std::string> &);

#endif // OPEN3DOR_HPP
