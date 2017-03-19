
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

void print_query_info(point3d query, SemanticOcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    cout << "color of node is: " << node->getColor() << endl;
    cout << "Semantics of node is: " << node->getSemantics() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
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
    
    // fuse extra information
    for (int i = 0; i < (int)cloud->size(); ++i) {
      const point3d& query = (*cloud)[i];
      std::vector<float> extra_info = cloud->getExtraInfo(i);
      SemanticOcTreeNode* n = tree.search (query);
      tree.averageNodeColor(n, extra_info[0], extra_info[1], extra_info[2]);
      //print_query_info(query, n);      
    }

    tree.write("semantic_color_scan.ot");
  }

  
  cout << "Test done." << endl;
  exit(0);

}
