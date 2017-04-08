#include <stdio.h>
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

#define NUMBER_LABELS 3
#define MAP_RESOLUTION 0.05
#define NUM_EXTRAINFO 4
#define SMOOTHFACTOR 0.9
#define FLOW_THRESHOLD 50

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

double randn (double mu, double sigma)
{
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
 
  if (call == 1)
    {
      call = !call;
      return (mu + sigma * (double) X2);
    }
 
  do
    {
      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);
 
  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
 
  call = !call;
 
  return (mu + sigma * (double) X1);
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
    random(i) = randn(0,1);
  }
  error = V*D*random;
//  cout << error << "Error" << endl;
  return error;
}

float calcNewOccupancy(double pl, double pg)
{
  double npl = 1.0-pl;
  double npg = 1.0-pg;

  // normalization
  double pg_new = pl*pg;
  double npg_new = npl*npg;
  pg_new = pg_new/(pg_new+npg_new);

  return octomap::logodds(pg_new);
}


std::vector<float> calNewSemantics(
    SemanticOcTreeNode::Semantics sl, SemanticOcTreeNode::Semantics sg) {
  std::vector<float> sg_new;
  for (int i = 0; i < (int)sl.label.size(); i++) {
    float s_new = sl.label[i] * sg.label[i];
    sg_new.push_back(s_new);
  }
  return sg_new;
}

//----------------------------------------------------main----------------------------------//

int main(int argc, char** argv) {
  if (argc < 2){
    printUsage(argv[0]);
  }


  // the global tree
  SemanticOcTree tree (MAP_RESOLUTION);


//--------------------------------------correction----------------------------------//

  // +++++++++++++++ START OF BUILDING LOCAL TREE +++++++++++++++++++++++ // 
  SemanticOcTree* localTree = new SemanticOcTree(MAP_RESOLUTION);
  float label[5][5] = {{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 0, 1}};
  int color[5][3] = {{255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};
  
  // prepare label for each class
  std::vector<std::vector<float> > labels;
  labels.resize(NUMBER_LABELS);
  for (int i = 0; i < NUMBER_LABELS; i++) {
    for (int j = 0; j < NUMBER_LABELS; j++)
      labels[i].push_back(label[i][j]);
    //cout<< labels[i] << endl;
  }
  
  // read frame pose from text file
  std::string filename = std::string(argv[1]);
  std::ifstream infile(filename.c_str());

  // read point cloud with extrainfo
  Pointcloud* cloud = new Pointcloud();
  while (infile) {
    cloud->readExtraInfo(infile, NUM_EXTRAINFO);
  }
  cout << "Finish reading the point cloud." << endl;

  // set point cloud origin pose for allignment
  pose6d origin(0, 0, 0, 0, 0, 0);
  
  // insert into OcTree 
  {
    // insert in global coordinates
    localTree->insertPointCloud(*cloud, origin.trans());
    cout << "Finish occupancy mapping." << endl;

    // add semantics to the point cloud
    for (int i = 0; i < (int)cloud->size(); i++) {
      const point3d& query = (*cloud)[i];
      std::vector<float> extra_info = cloud->getExtraInfo(i);
      SemanticOcTreeNode* n = localTree->search (query);
      localTree->averageNodeSemantics(n, labels[ int(extra_info[NUM_EXTRAINFO-1]) ]);
      //print_query_info(query, n);  
    }
  }

  // add semantics to all other nodes
  for (SemanticOcTree::iterator it = localTree->begin(); it != localTree->end(); ++it) {
    if ( !it->isSemanticsSet() ) {
      localTree->averageNodeSemantics((&(*it)), labels[0]);
    }
  }

  cout << "Finish building local TREE" << endl;
  
  // traverse the local tree, set color based on semantics to visualize
  for (SemanticOcTree::iterator it = localTree->begin(); it != localTree->end(); ++it) {
    if ( it->isSemanticsSet() ) {
      SemanticOcTreeNode::Semantics s = it->getSemantics();
      // Debug
      //print_query_info(point3d(0,0,0), &(*it)); 
      for (int i = 0; i < NUMBER_LABELS; i++) {
        if (s.label.size() && s.label[i] > 0.6) {
          it->setColor(color[i][0], color[i][1], color[i][2]);
        }
      }
    }
  }//end for

  localTree->write("local_tree.ot");

  // +++++++++++++++ END OF BUILDING LOCAL TREE +++++++++++++++++++++++ // 

  
  // +++++++++++++++ START TO UPDATE GLOBAL TREE +++++++++++++++++++++++ // 
  cout << "Expanded num. leafs: " << tree.getNumLeafNodes() << endl;
  // update the global tree according to local tree
  for (SemanticOcTree::leaf_iterator it = localTree->begin_leafs(),
      end = localTree->end_leafs(); it != end; ++it)
  {
    point3d queryCoord = it.getCoordinate();
    SemanticOcTreeNode* n = tree.search(queryCoord);
    double pl = it->getOccupancy();
    SemanticOcTreeNode::Semantics sl = it->getSemantics();
    
    if (n==NULL){
      // create a new node in global tree with the same coord
      // set the occupancy log odds
      float ol = octomap::logodds(pl);
      SemanticOcTreeNode* newNode = tree.updateNode(queryCoord, ol);
      // set the semantics
      newNode->setSemantics(sl);
      newNode->resetSemanticsCount();

    } else{
      // update occupancy prob according to Eq.(7) in paper
      double pg = n->getOccupancy();
      float og_new = calcNewOccupancy(pl, pg);
      n->setLogOdds(og_new);
      
      // if the global node does not have semantics, set as the local value
      if(!n->isSemanticsSet()) {
        OCTOMAP_ERROR("something wrong..");
        continue;
      }

      // else update semantics according to Eq.(7) in paper
      SemanticOcTreeNode::Semantics sg = n->getSemantics();
      std::vector<float> sg_new = calNewSemantics(sl, sg);
      // set new value
      n->setSemantics(sg_new);
      n->normalizeSemantics();
    }
  }
  cout << "Expanded num. leafs: " << tree.getNumLeafNodes() << endl;
  // +++++++++++++++ END OF UPDATING GLOBAL TREE +++++++++++++++++++++++ // 
  delete localTree;

//----------------------------------end correction----------------------------------//



//----------------------------------------------------prediction----------------------------------//
  SemanticOcTree* temp_tree = new SemanticOcTree(MAP_RESOLUTION);
  MatrixXf flowSigma(3, 3);
  flowSigma << 0.5, 0, 0,
               0, 0.5, 0,
               0, 0, 0.5;
//-----------------------------find the weights/number of particles----------------------------------//

   std::vector<SemanticOcTreeNode*> node_vec; // unique vector of voxels containing points
   std::vector<int> voxel_count;    // voxel counts for each voxel
   std::vector<int> point_voxel_map;// mapping vector from points to voxels
   std::vector<point3d> point_vec; //vector of one point in each voxel
   int pos=0;
   // cout<<"(int)sceneflow->size()"<<(int)sceneflow->size()<<endl;
  for (int i=0; i< (int)cloud->size(); ++i)
  {
    point3d& query = (*cloud)[i];
    SemanticOcTreeNode* n = tree.search(query);
    vector<SemanticOcTreeNode*>::iterator it;
    bool outofscreen= false;
    // check if scene flow is irregular
    std::vector<float> extra_info = cloud->getExtraInfo(i);
    point3d point_flow (extra_info[0], extra_info[1], extra_info[2]);
    if (point_flow.norm() > FLOW_THRESHOLD){
      //if norm of flow is too large== points moved out of screen
      outofscreen = true;
      // tree.deleteNode(point_flow(0),point_flow(1),point_flow(2));
    }


    it=find(node_vec.begin(),node_vec.end(),n);

    //if points in the new voxel
    if(it==node_vec.end()){
      node_vec.push_back(n);
      if (outofscreen){voxel_count.push_back(0);}
      else{voxel_count.push_back(1);}
      
      point_voxel_map.push_back((voxel_count.size())-1);


      point3d temp_node;
      for (int j=0;j<3;j++){
          temp_node(j) = query(j);
          // cout << "new_pos " << new_pos(j) << "  query  " << query(j) << "\n" << endl;
        }

      point_vec.push_back(temp_node);
       // cout<<"NOT FOUND"<<endl;
    }
    //if points in the old voxel
    else{
      pos = std::distance(node_vec.begin(), it);
      // cout<<pos<<endl;
      if (outofscreen){}
      else{voxel_count[pos]++;} 
      
      point_voxel_map.push_back(pos);
    }
  }


//-----------------------------second loop for all the points, propagate using sceneflow----------------------------------//

int nop = 10; // Nuumber of particles
// sf_cloud is assumed to contain infor about sceneflow too
for (int i=0; i< (int)cloud->size(); ++i)
{
  const point3d& query = (*cloud)[i];
  std::vector<float> extra_info = cloud->getExtraInfo(i);
  point3d point_flow (extra_info[0], extra_info[1], extra_info[2]);

  // filter out some bad values for scene flow
  if (point_flow.norm() > FLOW_THRESHOLD)
  {
    //OCTOMAP_ERROR("scene flow may be a bad value.\n");
    continue;
  }

  SemanticOcTreeNode* n = tree.search (query);
  if(!n)
  {
    OCTOMAP_ERROR("something wrong..");
    continue;
  }
  
  SemanticOcTreeNode::Semantics s = n->getSemantics();
  
  double occ = n->getOccupancy();
  // cout<<"occ->"<<occ<<endl;
  //	SIMALRLY GET OCTREENode OCCUPANCY AND STORE IT
  //  Need to get corrsponding weights
  
  std::vector<float> label = s.label;
  //    std::cout << label << std::endl;
  // Get weight
  
  int weight = nop*(voxel_count[point_voxel_map[i]]); 
  
  // cout<<"weight->"<<weight<<endl;
  //	Need to verify this
  std:: vector<float> new_so;
  for(int m = 0; m < (int)label.size();m++){
    new_so.push_back(label[m]/weight); 
  }

  new_so.push_back(occ/weight); // OCC IS PROBABILITY OF OCCUPIED
  new_so.push_back((1-occ)/weight);

  // //debug
  // cout<<"new_so->"<<endl;
  // for (int i = 0; i < new_so.size(); ++i)
  // {
  //   cout<<new_so[i]<<endl;
  // }


  for (int k = 0; k < nop ; k++){
      point3d new_pos;
      VectorXf error = sampleFlow(flowSigma);// Need to get flowSigma

      for (int j=0;j<3;j++){
        new_pos(j) = query(j) + point_flow(j) + error(j);
        //cout << "new_pos " << new_pos(j) << "  query  " << query(j) << "\n" << endl;
      }
  
      float ol = 10.0;
      
      SemanticOcTreeNode* newNode = temp_tree->search(new_pos);
  // If node doesn't exist we add the semantics combined with occcupancy directly.        
  
    if(newNode == NULL){
        SemanticOcTreeNode* newNode = temp_tree->setNodeValue(new_pos, ol);
        newNode->setSemantics(new_so); // Need to check this 
    
  //		We need to get occupancy from Octree. NOT SURE OF COMMAND
        //tree.averageNodeSemantics(newNode, label);
  //        //print_query_info(query, n);  
      }
    else{
      SemanticOcTreeNode::Semantics curr_so = newNode->getSemantics();
      vector<float> temp;
      for(int m = 0;m < (int)curr_so.label.size();m++){
        temp.push_back(curr_so.label[m]+new_so[m]);
      }
      newNode -> setSemantics(temp);
    }
  }
}


//------------------------- delete the voxel that has been updated in orginal tree----------------------------------//

for (int i=0; i< (int)point_vec.size(); ++i){
  tree.deleteNode(point_vec[i](0),point_vec[i](1),point_vec[i](2));
}


//------------------------- smoothing and normalize----------------------------------//

for (SemanticOcTree::iterator it = temp_tree->begin(); it != temp_tree->end(); ++it) {
  
  SemanticOcTreeNode::Semantics s = it->getSemantics();
	vector<float> lo = s.label;
	vector<float> occupancy;
	vector<float> labels;
	vector<float> upd_labels;
  vector<float> upd_occupancy;
	for(int m = 0; m < NUMBER_LABELS;m++){// Define NUMBER_LABELS
		labels.push_back(lo[m]);
	}
	occupancy.push_back(lo[NUMBER_LABELS]);
	occupancy.push_back(lo[NUMBER_LABELS+1]);
	


	//labels smoothening
	float lab_sf = SMOOTHFACTOR;
	float lab_sf_o = (1 - lab_sf)/(labels.size() - 1);
	float norm_sum = 0;
  float new_sum = 0;
	for(int m = 0;m< (int)labels.size();m++){
		float sum = 0;
		for(int n = 0;n <(int)labels.size();n++){
			if(m == n){
				sum = sum + (labels[n]*lab_sf);
			}
			else{
				sum = sum + (labels[n]*lab_sf_o);
			}
		}
		norm_sum = norm_sum + sum;
		upd_labels.push_back(sum);
    // upd_labels.push_back(labels[m]);
    // new_sum = new_sum + labels[m];
	}
	// cout<<"upd_labels->"<<endl;
  for(int m = 0; m < (int)labels.size(); m++)
    {
      upd_labels[m] /= norm_sum;
      // upd_labels[m] = upd_labels[m]/new_sum;
      // cout<<upd_labels[m]<<" "<<endl;
    }
  
  
  //occupancy smoothening
  float occ_sf = SMOOTHFACTOR;
	float occ_sf_o = 1 - occ_sf;
  norm_sum = 0;
  new_sum = 0;
	for(int m = 0; m<2;m++){
		float sum = 0;
		for(int n = 0;n < 2;n++){
			if(m == n){
				sum = sum + (occupancy[n]*occ_sf);
			}
			else{
				sum = sum + (occupancy[n]*occ_sf_o);
			}
		}
		norm_sum = norm_sum + sum;
		upd_occupancy.push_back(sum);
    // upd_occupancy.push_back(occupancy[m]);
    // new_sum = new_sum + occupancy[m];
	}
  // cout<<"occupancy->"<<endl;
	for(int m = 0; m < 2; m++)
    {
      upd_occupancy[m] /= norm_sum;
      // upd_occupancy[m] = upd_occupancy[m]/new_sum;
      // cout<<upd_occupancy[m]<<" "<<endl;
    }

  // update the temp tree
  it->setLogOdds(octomap::logodds(upd_occupancy[0]));
  it->setSemantics(upd_labels);

}

//------------------------- update the original tree----------------------------------//
for (SemanticOcTree::leaf_iterator it = temp_tree->begin_leafs(),
    end = temp_tree->end_leafs(); it != end; ++it)
{
  point3d queryCoord = it.getCoordinate();  
  SemanticOcTreeNode* n = tree.search(queryCoord);
  float ol = it->getLogOdds();
  SemanticOcTreeNode::Semantics sl = it->getSemantics();
  // update occupancy and semantics
    // cout<<"ol->"<<ol<<endl;
  
  if(!n) {
    SemanticOcTreeNode* new_node = tree.updateNode(queryCoord,ol);
    new_node->setSemantics(sl);

  //   //debug
  //   cout<<"coordinates->";
  //   for (int i = 0; i < 3; ++i)
  //   {
  //     cout<<queryCoord(i)<<endl;
  //   }  

  //   SemanticOcTreeNode::Semantics s = new_node->getSemantics();
  //   vector<float> lo = s.label;
  //   vector<float> occupancy;
  //   cout<<"label->";
  //   for (int i = 0; i < 3; ++i)
  //   {
  //     cout<<lo[i]<<endl;
  //   }  

  //   cout<<"occupancy->"<< new_node->getOccupancy()<<endl;

  }
  else{

  //   cout<<"old node!!!!"<<endl;

  n->setLogOdds(octomap::logodds(ol));
  n->setSemantics(sl);
  }


  // debug
}
delete temp_tree;
//----------------------------------------------------end prediction----------------------------------//



  // Debug: test if the update is correct
  for (SemanticOcTree::leaf_iterator it = localTree->begin_leafs(),
      end = localTree->end_leafs(); it != end; ++it)
  {
    SemanticOcTreeNode* n = tree.search(it.getCoordinate());
    if (n==NULL){
      //OCTOMAP_ERROR("something wrong...\n");
    } else{
      //cout << it->getOccupancy() << " " << n->getOccupancy() << endl;
      //if(n->isSemanticsSet())
        //cout << it->getSemantics().label << it->getSemantics().count << " "
          //<< n->getSemantics().label << n->getSemantics().count << endl;
    }
  }
 


  // traverse the whole tree, set color based on semantics to visualize
  for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
    if ( it->isSemanticsSet() ) {
      SemanticOcTreeNode::Semantics s = it->getSemantics();
      // Debug
      //print_query_info(point3d(0,0,0), &(*it)); 
      for (int i = 0; i < NUMBER_LABELS; i++) {
        if (s.label.size() && s.label[i] > 0.6) {
          it->setColor(color[i][0], color[i][1], color[i][2]);
        }
      }
    }
  }//end for


  tree.write("global_tree.ot");

  cout << "Test done." << endl;
  exit(0);

}
