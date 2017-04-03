
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"
#include <eigen3/Eigen/Core>

using namespace std;
using namespace octomap;
using namespace Eigen;

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
  float labels[5][5]= {{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}};
  int color[5][3] = {{255, 0, 0}, {0, 255, 0}};
 

  for ( int argn = 0; argn < argc-1; argn++) {
    // read frame pose from text file
    std::string filename = std::string(argv[argn+1]);
    std::ifstream infile(filename.c_str());
    
    // prepare label for current class
    std::vector<float> label;
    for (int c = 0; c < argc-1; c++) {
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

	// Prediction
	
	SemanticOcTree temp_tree (0.05);
	Pointcloud* new_cloud = new Pointcloud();
	std::string filename = std::string(argv[1]);
	std::ifstream infile(filename.c_str());
	while (infile) {
	  new_cloud->readExtraInfo(infile, 3);
	}
	float sceneflow[3] = {1.0, 1.0, 1.0};
	//MatrixXf flowSigma[3][3] = {{2.0,0,0},{0,2,0},{0,0,2}};
	MatrixXf flowSigma(3,3);
	MatrixXf V;
	MatrixXf S;
	flowSigma << 1,0,0,
				0,2,0,
				0,0,1;
	
	
	JacobiSVD<MatrixXf> svd(flowSigma, ComputeThinU | ComputeThinV);
	V = svd.matrixV(); // need to define
	S = svd.singularValues();
	float D[3][3] = {{0,0,0},{0,0,0},{0,0,0}}; 
	for (int i = 0; i < 3;i ++){
		D[i][i] = sqrt(S(i));
	}
	float random[3] = {rand(),rand(),rand()};
	float error[3];
	for (int i = 0;i < 3;i++){
		error[i] = V(i,1)*D[1][1]*random[1] + V(i,2)*D[2][2]*random[2] + V(i,3)*D[3][3]*random[3]; 
	}
	for (int i=0; i< (int)new_cloud->size(); ++i)
	{
	  const point3d& query = (*new_cloud)[i];
	  SemanticOcTreeNode* n = tree.search (query);
	  SemanticOcTreeNode::Semantics s = n->getSemantics();
	  std::vector<float> label = s.label;
	  std::cout << label << std::endl;	
	  const point3d& new_pos;
	  for (int j=0;j<3;j++){
		  new_pos[j] = query[j] + sceneflow[i] + error[i];
	  }   ////
//	  int np = sampleFlow();
	}


  // traverse the whole tree, set color based on semantics to visualize
  for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
    if ( (&(*it))->isSemanticsSet() ) {
      SemanticOcTreeNode::Semantics s = (&(*it))->getSemantics();
      // Debug
      //print_query_info(point3d(0,0,0), &(*it));  
      for (int argn = 0; argn < argc-1; argn++) {
        if (s.label.size() && s.label[argn] > 0.3) {
          (&(*it))->setColor(color[argn][0], color[argn][1], color[argn][2]);
        }
      }
    }
  }//end for


  tree.write("semantic_color_scan.ot");
  

  cout << "Test done." << endl;
  exit(0);

}
