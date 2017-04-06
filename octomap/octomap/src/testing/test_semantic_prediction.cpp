
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <time.h>       /* time */
#include <algorithm>    // std::find
#include <vector>       // std::vector

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


VectorXf sampleFlow(MatrixXf flowSigma){

  MatrixXf V;
  MatrixXf S;
  MatrixXf D(3,3);
  D << 0,0,0,
     0,0,0,
    0,0,0;
  VectorXf random(3);
  VectorXf error;
  JacobiSVD<MatrixXf> svd(flowSigma, ComputeThinU | ComputeThinV);
  V = svd.matrixV(); // need to define
  S = svd.singularValues();
  
  
  for (int i = 0; i < 3;i ++){
    D(i,i) = sqrt(S(i));
    random(i) = (double) rand() / (RAND_MAX);
  }
  error = V*D*random;
//  cout << error << "Error" << endl;
  return error;
}




int main(int argc, char** argv) {
  if (argc < 2){
    printUsage(argv[0]);
  }
  srand (time(NULL));
  
  SemanticOcTree tree (0.05);
  float labels[5][5]= {{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}};
  int color[5][3] = {{255, 0, 0}, {0, 255, 0}};
 
  Pointcloud* new_cloud = new Pointcloud();
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
    
    while (infile) {
      new_cloud->readExtraInfo(infile, 3);
    }

    // set point cloud origin pose for allignment
    pose6d origin(0, 0, 0, 0, 0, 0);
    
    // insert into OcTree  
    {
      // insert in global coordinates:
      tree.insertPointCloud(*new_cloud, origin.trans());
      
      // fuse extra information
      for (int i=0; i < (int)new_cloud->size(); ++i) {
        const point3d& query = (*new_cloud)[i];
        //std::vector<float> extra_info = cloud->getExtraInfo(i);
        SemanticOcTreeNode* n = tree.search (query);
        tree.averageNodeSemantics(n, label);
        //print_query_info(query, n);  
      }
    }
  }//end for

// read in the scene flow
  
  SemanticOcTree temp_tree (0.05);
  pose6d origin(0, 0, 0, 0, 0, 0);
  
  Pointcloud* sceneflow = new Pointcloud();
  
  std::string filename = std::string(argv[1]);
  std::ifstream infile(filename.c_str());
  while (infile) {
    sceneflow->readExtraInfo(infile, 3);
  }
  // temp_tree.insertPointCloud(*sceneflow, origin.trans());
    

//find the weights/number of particles
   std::vector<SemanticOcTreeNode*> node_vec;
   std::vector<int> voxel_count;
   std::vector<int> point_voxel_map;
   int pos=0;
  for (int i=0; i< (int)sceneflow->size(); ++i)
  {
    const point3d& query = (*new_cloud)[i];
    SemanticOcTreeNode* n = tree.search(query);
    vector<SemanticOcTreeNode*>::iterator it;

    it=find(node_vec.begin(),node_vec.end(),n);

    //if points in the new voxel
    if(it==node_vec.end()){
      node_vec.push_back(n);
      voxel_count.push_back(1);
      point_voxel_map.push_back((voxel_count.size())-1);
       // cout<<"NOT FOUND"<<endl;
    }
    //if points in the old voxel
    else{
      pos = std::distance(node_vec.begin(), it);
      // cout<<pos<<endl;
      voxel_count[pos]++;
      point_voxel_map.push_back(pos);
    }
  }


//second loop for all the points, propagate using sceneflow
int nop = 10; // Nuumber of particles
// sf_cloud is assumed to contain infor about sceneflow too
  for (int i=0; i< (int)new_cloud->size(); ++i)
  {
    const point3d& query = (*new_cloud)[i];
    const point3d& point_flow = (*sceneflow)[i];
    SemanticOcTreeNode* n = tree.search (query);
    SemanticOcTreeNode::Semantics s = n->getSemantics();
//	SIMALRLY GET OCTREENode OCCUPANCY AND STORE IT
//  Need to get corrsponding weights
    std::vector<float> label = s.label;
//    std::cout << label << std::endl;
	// Get weight
//	int weight = nop*(point_voxel_map[i]) 
//	Need to verify this
	std:: vector<float> new_so;
	for(int m = 0; m < label.size();m++){
		new_so.push_back(label[m]/weight); 
	}
	new_so.push_back(occ/weight); // OCC IS PROBABILITY OF OCCUPIED
	new_so.push_back((1-occ)/weight);
	
    for (int k = 0; k < nop ; k++){
        
        point3d new_pos;
        VectorXf error = sampleFlow(flowSigma);// Need to get flowSigma

        for (int j=0;j<3;j++){
          new_pos(j) = query(j) + point_flow(j) + error(j);
          // cout << "new_pos " << new_pos(j) << "  query  " << query(j) << "\n" << endl;
        }
    
        float ol = 10.0;
        
        SemanticOcTreeNode* newNode = temp_tree.search(new_pos);
		// If node doesn't exist we add the semantics combined with occcupancy directly.        
		
		if(newNode == NULL){
        SemanticOcTreeNode* newNode = temp_tree.setNodeValue(new_pos, ol);
        
		newNode->setSemantics(new_so); // Need to check this 
		
//		We need to get occupancy from Octree. NOT SURE OF COMMAND
        //tree.averageNodeSemantics(newNode, label);
//        //print_query_info(query, n);  
  
        }
        else{
			SemanticOcTreeNode::Semantics curr_so = newNode -> getSemantic();
			vector<float> temp;
			for(int m = 0;m < curr_so.size();m++){
				temp.push_back(curr_so[m]+new_so[m]);
			}
			newNode -> setSemantics(temp);
        }
    }
  }
  
  


//store in temp tree. Delete voxels 





//smoothing

for (int i=0; i< (int)new_cloud->size(); ++i){
	const point3d& query = (*new_cloud)[i];
    SemanticOcTreeNode* n = temp_tree.search (query);
    SemanticOcTreeNode::Semantics s = n->getSemantics();
	SemanticOcTreeNode::Semanti
}
  

//normalize



 

//update the original tree
for (SemanticOcTree::leaf_iterator it = temp_tree->begin_leafs(),
    end = temp_tree->end_leafs(); it != end; ++it)
{
  point3d queryCoord = it.getCoordinate();
    
  // update occupancy
  double pl = it->getOccupancy();
  SemanticOcTreeNode* n = tree.setLogOdds(queryCoord, octomap::logodds(pl));
  
  // update semantics
  SemanticOcTreeNode::Semantics sl = it->getSemantics();
  n->setSemantics(sl);

  // debug
  //cout<< it->getOccupancy() << " " << n->getOccupancy() << " "
    //<< it->getSemantics().label << n->getSemantics().label << endl;
}


//	
//FOR SANITY 
  
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


  tree.write("semantic_color_scan_Moved.ot");



  

  cout << "Test done." << endl;
  exit(0);

}


//  
