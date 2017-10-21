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

		meshes.push_back(m);
  }

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "\e[1;34mMeshes loaded to database: " << meshes.size() << "\n";
  std::cout << "End loading Database\nElapsed time: " << elapsed_secs
            << "\e[0m\n";
}

void preprocessDB(std::vector<Mesh> &meshes) {
	std::cout << "\e[1;34mStart preprocessing Database\n";

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

	std::vector<Mesh> processed_meshes;

	for (int i = 0; i < meshes.size(); ++i) {
		meshes[i].setGridSize(0.05);
		processed_meshes.push_back(meshes[i].gridFilter());
		//processed_meshes.push_back(meshes[i]);
		processed_meshes[i].process();
	}
	
	meshes = processed_meshes;

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "End preprocessing Database\nElapsed time: " << elapsed_secs
            << "\e[0m\n";
}
