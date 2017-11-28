#include "Database.hpp"

void loadDB(std::string db_path, std::vector<Mesh> &meshes) {
	std::cout << "\e[1;34mStart loading Database: " << db_path << "\n";

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

	boost::filesystem::path p(db_path);

  for (boost::filesystem::directory_iterator dir(p);
       dir != boost::filesystem::directory_iterator(); dir++) {

    std::string db_entry = (*dir).path().string();
    if (db_entry.substr(db_entry.size() - 3, 3) != "obj" && 
				db_entry.substr(db_entry.size() - 3, 3) != "off")
      continue;

    Mesh m;
		m.load(db_entry);
		m.preprocess();
		m.computeNormals();

		meshes.push_back(std::move(m));
  }

//	std::sort(meshes.begin(), meshes.end(), [] (const Mesh &a, const Mesh &b)
//			-> bool { return a.getID() < b.getID();});

//	for (auto &m : meshes)
//		std::cout << m.getID() << "\n";

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "\e[1;34mMeshes loaded to database: " << meshes.size() << "\n";
  std::cout << "End loading Database\nElapsed time: " << elapsed_secs
            << "\e[0m\n";
}

void preprocessDB(std::vector<Mesh> &meshes, std::vector<Mesh> &processed_meshes) {
	if (!processed_meshes.empty()) {
		processed_meshes.clear();
		processed_meshes.shrink_to_fit();
	}

	std::cout << "\e[1;34mStart preprocessing Database\n";

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

	for (int i = 0; i < meshes.size(); ++i) {
		//meshes[i].setGridSize(0.08);
		processed_meshes.push_back(std::move(meshes[i].gridFilter()));
		//processed_meshes.push_back(meshes[i]);
		processed_meshes[i].process();
	}
	
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "End preprocessing Database\nElapsed time: " << elapsed_secs
            << "\e[0m\n";
}

void readObjectClasses(std::string path, std::vector<Mesh> &meshes) {
	std::ifstream class_file;
	std::string line;

	class_file.open(path);
	if (class_file.is_open()) {
		while (getline(class_file, line)) {
			std::istringstream in(line);
			int obj_class;
			int obj_idx;

			in >> obj_class;
			in >> obj_idx;

			//std::cout << "Object : " << obj_idx << " belongs to class : " << obj_class << "\n";
			//std::cout << "ID : " << meshes[obj_idx - 1].getID() << " belongs to class : " << obj_class << "\n";

			meshes[obj_idx - 1].setClass(obj_class);
		}
	}
	class_file.close();
	printClassesHistogram(meshes);
}

void printClassesHistogram(std::vector<Mesh> &meshes) {
	std::vector<int> histogram(6, 0);

	for (auto &m : meshes) 
		histogram[m.getClass() - 1]++;

	for (int i = 0; i < histogram.size(); ++i)
		std::cout << "Class " << i + 1 << " has " << histogram[i] << " objects\n";

	for (auto i : histogram) {
		std::cout << "\e[1;31;43m";
		for (auto j : std::vector<int>(i))
			std::cout << " ";
		std::cout << "\e[0m\n";
	}
}

void takePartialView(std::vector<Mesh> &meshes, float percentage) {
	Mesh partial_mesh;

	for (int i = 0; i < meshes.size(); ++i) {
		partial_mesh = meshes[i].partialView(percentage);
		partial_mesh.exportOff(std::to_string(meshes[i].getID()));
	}
}
