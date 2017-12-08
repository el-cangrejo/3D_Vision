#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <boost/filesystem.hpp>

#include "Mesh.hpp"

void loadDB(std::string db_path, std::vector<Mesh> &meshes);

void preprocessDB(std::vector<Mesh> &meshes, std::vector<Mesh> &processed_meshes);

void writeDBDescriptors(std::vector<Mesh> &meshes);

void readDBDescriptors(std::vector<Mesh> &meshes);

void readObjectClasses(std::string path, std::vector<Mesh> &meshes);

std::vector<int> printClassesHistogram(std::vector<Mesh> &);

void takePartialView(std::vector<Mesh> &, float);

std::vector<float> precisionRecall(Mesh &, std::vector<Mesh> &, int );

void precisionRecall(std::vector<Mesh> &, std::vector<Mesh> &);

void retrieve(Mesh &, std::vector<Mesh> &);
#endif //DATABASE_HPP
