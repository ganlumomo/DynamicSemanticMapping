
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " 5 x point_cloud.txt  (point cloud file, required)\n\n";

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
  if (argc < 2){
    printUsage(argv[0]);
  }

  
  SemanticOcTree tree (0.05);
  float labels[5][5]= {{0.3, 0.2, 0, 0, 0}, {0.2, 0.3, 0, 0, 0}, {0, 0, 0.8, 0, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 0, 1}};
  int color[5][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 20, 147}, {0, 255, 255}};
 

  for ( int argn = 0; argn < argc-1; argn++) {
    // read frame pose from text file
    std::string filename = std::string(argv[argn+1]);
    std::ifstream infile(filename.c_str());
    
    // prepare label for current class
    std::vector<float> label;
    for (int c = 0; c < argc; c++) {
      label.push_back(labels[argn][c]);
    }

    // read point cloud with extrainfo
    Pointcloud* cloud = new Pointcloud();
    while (infile) {
      cloud->readExtraInfo(infile, 3);
    }

    // set point cloud origin pose for allignment
    pose6d origin(0, 0, 0, 0, 0, 0);
    
    // insert into OcTree  
    {
      // insert in global coordinates:
      tree.insertPointCloud(*cloud, origin.trans());
      
      // fuse extra information
      for (int i=0; i < (int)cloud->size(); ++i) {
        const point3d& query = (*cloud)[i];
        //std::vector<float> extra_info = cloud->getExtraInfo(i);
        SemanticOcTreeNode* n = tree.search (query);
        tree.averageNodeSemantics(n, label);
        //print_query_info(query, n);  
      }
    }
  }//end for

  // add semantics to all other nodes
  std::vector<float> other_label;
  for (int c = 0; c < argc; c++){
    other_label.push_back(labels[argc-1][c]);
  }
  for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
    if ( !it->isSemanticsSet() ) {
      tree.averageNodeSemantics((&(*it)), other_label);
    }
  }





  // traverse the whole tree, set color based on semantics to visualize
  for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
    if ( it->isSemanticsSet() ) {
      SemanticOcTreeNode::Semantics s = it->getSemantics();
      //cout << s.label << s.count << endl;
      // Debug
      //print_query_info(point3d(0,0,0), &(*it));  
      for (int argn = 0; argn < argc-1; argn++) {
        if (s.label.size() && s.label[argn] > 0.5) {
          it->setColor(color[argn][0], color[argn][1], color[argn][2]);
        }
      }
    }
    else {
      OCTOMAP_ERROR("Some voxels DO NOT have semantics.");
    }
  }//end for


  tree.write("semantic_color_scan.ot");
  

  cout << "Test done." << endl;
  exit(0);

}
