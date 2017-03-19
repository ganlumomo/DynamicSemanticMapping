
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " point_cloud.txt  (point cloud file, required)\n\n";

  exit(1);
}

int main(int argc, char** argv) {
  if (argc != 2){
    printUsage(argv[0]);
  }

  std::string filename = std::string(argv[1]);

  Pointcloud* cloud = new Pointcloud();
  
  // read frame pose from text file
  std::ifstream infile(filename.c_str());
  
  while (infile) {
    cloud->readExtraInfo(infile, 3);
  }
  
  
  pose6d origin(0, 0, 0, 0, 0, 0);
  
  // insert into OcTree  
  {
    SemanticOcTree tree (0.05);  

    // insert in global coordinates:
    tree.insertPointCloud(*cloud, origin.trans());

    tree.write("semantic_scan.ot");
  }
  
  cout << "Test done." << endl;
  exit(0);

}
