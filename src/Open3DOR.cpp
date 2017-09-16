#include "Open3DOR.hpp"

#include <armadillo>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>

int loadObj(const std::string filepath, Mesh &mesh) {
  if (filepath.substr(filepath.size() - 3, 3) != "obj") {
    std::cout << "Wrong file format!\n";
    return 0;
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
  return 1;
}

void read_mesh(const std::string filepath, Mesh &mesh) {
    if (filepath.substr(filepath.size() - 3, 3) == "obj") {
      loadObj(filepath, mesh);
    } else if (filepath.substr(filepath.size() - 3, 3) == "off") {
      loadOff(filepath, mesh);
    } else {
			std::cout << "FIle format not supported (yet)\n";
      return;
		}
}

int loadOff(const std::string filepath, Mesh &mesh) {
	std::ifstream off_file;
  std::string line;
  float maxdist(0.0);

  std::vector<Vertex> std_normals;
  off_file.open(filepath);

  if (off_file.is_open()) {
    std::cout << "Started reading " << filepath << "\n";
    getline(off_file, line);
    getline(off_file, line);

    std::istringstream in(line);
    int v, f, e;
    in >> v >> f >> e;

    for (int i = 0; i < v; ++i) {
        getline(off_file, line);
        std::istringstream in_v(line);
        float x, y, z;
        in_v >> x >> y >> z;

        mesh.centroid.x += x;
      mesh.centroid.y += y;
      mesh.centroid.z += z;

        mesh.vertices.push_back(Vertex(x, y, z));
    }
    for (int i = 0; i < f; ++i) {
        getline(off_file, line);
        std::istringstream in_f(line);
        int x, y, z;
        int n;
        in_f >> n;
        in_f >> x >> y >> z;

        mesh.triangles.push_back(Triangle(x, y, z));
    }
    off_file.close();
  } else {
    std::cout << "Unable to open file! \n";
    return 0;
  }

  mesh.centroid.x /= mesh.vertices.size();
  mesh.centroid.y /= mesh.vertices.size();
  mesh.centroid.z /= mesh.vertices.size();
  return 1;
}

void preprocess_mesh(Mesh &mesh) {
  mesh.movetoCenter();
  mesh.fittoUnitSphere();
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
    target_mesh.fpfhist = load_descriptors(target_file);
    std::string target_obj =
        target_file.substr(0, target_file.size() - 3) + "obj";
    db_fil.push_back(target_obj);
    // std::cout << "Hist size : " << target_mesh.fpfhist[0].size() << "\n";
    db_descr.push_back(target_mesh);
    ++count;
  }
  std::cout << "Files preprocessed in database : " << count << "\n";
}


