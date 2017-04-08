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

#define MAP_RESOLUTION 0.05
#define NUM_LABELS 3
#define NUM_EXTRAINFO 4
#define NUM_PARTICLES 10
#define SMOOTH_FACTOR 1.0
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

double randn (double mu, double sigma) {
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
 
  if (call == 1) {
    call = !call;
    return (mu + sigma * (double) X2);
  }
 
  do {
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

VectorXf sampleFlow(MatrixXf flowSigma) {
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
  
  for (int i = 0; i < 3; i++) {
    D(i,i) = sqrt(S(i));
    random(i) = randn(0,1);
  }
  error = V*D*random;
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

  // build the global tree
  SemanticOcTree tree (MAP_RESOLUTION);

  float label[5][5] = {{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 0, 1}};
  int color[5][3] = {{255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};
  
  // prepare label for each class
  std::vector<std::vector<float> > labels;
  labels.resize(NUM_LABELS);
  for (int i = 0; i < NUM_LABELS; i++) {
    for (int j = 0; j < NUM_LABELS; j++)
      labels[i].push_back(label[i][j]);
  }

//-------------------------------------------the very big loop----------------------------------//

  for (int argn = 1; argn < argc; argn++) {
    
    //---------------------read the current frame data------------------//
    
    // read frame pose from text file
    std::string filename = std::string(argv[argn]);
    std::ifstream infile(filename.c_str());
    
    // read point cloud with extrainfo
    Pointcloud* cloud = new Pointcloud();
    // set point cloud origin pose for allignment
    pose6d origin(0, 0, 0, 0, 0, 0);
   
    while (infile) {
      cloud->readExtraInfo(infile, NUM_EXTRAINFO);
    }
    cout << "Finish reading the point cloud." << endl;



    //----------------------------------------------------prediction----------------------------------//
    
    //---------------------start to build the local tree------------------//
    
    SemanticOcTree* localTree = new SemanticOcTree(MAP_RESOLUTION);
    
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

    // add semantics to all other nodes
    for (SemanticOcTree::iterator it = localTree->begin(); it != localTree->end(); ++it) {
      if ( !it->isSemanticsSet() ) {
        localTree->averageNodeSemantics((&(*it)), labels[0]);
      }
    }
    cout << "Finish building local tree." << endl;
    
    // traverse the local tree, set color based on semantics to visualize
    for (SemanticOcTree::iterator it = localTree->begin(); it != localTree->end(); ++it) {
      if ( it->isSemanticsSet() ) {
        SemanticOcTreeNode::Semantics s = it->getSemantics();
        // Debug
        //print_query_info(point3d(0,0,0), &(*it)); 
        for (int i = 0; i < NUM_LABELS; i++) {
          if (s.label.size() && s.label[i] > 0.6) {
            it->setColor(color[i][0], color[i][1], color[i][2]);
          }
        }
      }
    } //end for
    std::string outfile = std::string(argv[argn]) + "_local_tree.ot";
    localTree->write(outfile.c_str());

    //---------------------end of building the local tree------------------//
   

    //---------------------start to update the global tree------------------//
    
    cout << "Expanded num. leafs before: " << tree.getNumLeafNodes() << endl;
    // update the global tree according to local tree
    for (SemanticOcTree::leaf_iterator it = localTree->begin_leafs(),
        end = localTree->end_leafs(); it != end; ++it)
    {
      point3d queryCoord = it.getCoordinate();
      SemanticOcTreeNode* n = tree.search(queryCoord);
      double pl = it->getOccupancy();
      SemanticOcTreeNode::Semantics sl = it->getSemantics();
      
      if(!n) {
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
    cout << "Expanded num. leafs after: " << tree.getNumLeafNodes() << endl;
    delete localTree;
    
    // traverse the whole tree, set color based on semantics to visualize
    for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
      if ( it->isSemanticsSet() ) {
        SemanticOcTreeNode::Semantics s = it->getSemantics();
        // Debug
        //print_query_info(point3d(0,0,0), &(*it)); 
        for (int i = 0; i < NUM_LABELS; i++) {
          if (s.label.size() && s.label[i] > 0.6) {
            it->setColor(color[i][0], color[i][1], color[i][2]);
          }
        }
      }
    } //end for
    outfile = std::string(argv[argn]) + "_after_correction.ot";
    tree.write(outfile);

    //---------------------end of updating the global tree------------------//
 
    //----------------------------------------------------end correction----------------------------------//

  

    //----------------------------------------------------prediction----------------------------------//
    
    SemanticOcTree* temp_tree = new SemanticOcTree(MAP_RESOLUTION);
    MatrixXf flowSigma(3, 3);
    flowSigma << 0.01, 0, 0,
                 0, 0.01, 0,
                 0, 0, 0.01;

    // find voxel weights according to points
    for (int i=0; i<(int)cloud->size(); i++) {
      const point3d& query = (*cloud)[i];
      SemanticOcTreeNode* n = tree.search(query);
      if(n)
        n->addSemanticsCount();
    }

 
    //-----------------------------second loop for all the points, propagate using sceneflow----------------------------------//

    for (int i=0; i<(int)cloud->size(); i++)
    {
      const point3d& query = (*cloud)[i];
      std::vector<float> extra_info = cloud->getExtraInfo(i);
      point3d point_flow (extra_info[0], extra_info[1], extra_info[2]);

      // filter out some bad values for scene flow
      if (point_flow.norm() > FLOW_THRESHOLD) {
        //OCTOMAP_ERROR("scene flow may be a bad value.\n");
        continue;
      }

      // get the particle info from the original voxel
      SemanticOcTreeNode* n = tree.search (query);
      if(!n) {
        OCTOMAP_ERROR("something wrong..");
        continue;
      }
      
      SemanticOcTreeNode::Semantics s = n->getSemantics();
      double occ = n->getOccupancy();
      int weight = NUM_PARTICLES*(s.count);

      // prepare new semantics and occupancy for temp tree voxel
      std:: vector<float> new_so;
      for (int m = 0; m < (int)s.label.size(); m++)
        new_so.push_back(s.label[m]/weight);
      new_so.push_back(occ/weight); // OCC IS PROBABILITY OF OCCUPIED
      new_so.push_back((1-occ)/weight);

      for (int k = 0; k < NUM_PARTICLES; k++) {
        point3d new_pos;
        VectorXf error = sampleFlow(flowSigma);// Need to get flowSigma
        
        for (int j = 0; j < 3; j++) {
          new_pos(j) = query(j) + point_flow(j) + error(j);
          //cout << "new_pos " << new_pos(j) << "  query  " << query(j) << "\n" << endl;
        }
          
        // sum the particle info to temp tree voxel
        SemanticOcTreeNode* n = temp_tree->search(new_pos);
        if(!n) {
          SemanticOcTreeNode* newNode = temp_tree->setNodeValue(new_pos, 10);
          newNode->setSemantics(new_so);
        } else {
          SemanticOcTreeNode::Semantics curr_so = n->getSemantics();
          for (int m = 0; m < (int)curr_so.label.size(); m++) {
            new_so[m] += curr_so.label[m];
          }
          n->setSemantics(new_so);
        }
      }
    }
 

    //------------------------- smoothing and normalize----------------------------------//

    for (SemanticOcTree::iterator it = temp_tree->begin(); it != temp_tree->end(); it++) {
      SemanticOcTreeNode::Semantics s = it->getSemantics();
      
      vector<float> labels;
      vector<float> occupancy;
      vector<float> upd_labels;
      vector<float> upd_occupancy;
      for(int m = 0; m < NUM_LABELS; m++) {
        labels.push_back(s.label[m]);
      }
      occupancy.push_back(s.label[NUM_LABELS]);
      occupancy.push_back(s.label[NUM_LABELS+1]);
    

      //labels smoothening
      float lab_sf = SMOOTH_FACTOR;
      float lab_sf_o = (1 - lab_sf)/(labels.size() - 1);
      float norm_sum = 0;
      for (int m = 0; m < (int)labels.size(); m++) {
        float sum = 0;
        for (int n = 0; n < (int)labels.size(); n++) {
          if(m == n)
            sum += labels[n]*lab_sf;
          else
            sum += labels[n]*lab_sf_o;
        }
        norm_sum += sum;
        upd_labels.push_back(sum);
      }
      for (int m = 0; m < (int)labels.size(); m++)
        upd_labels[m] /= norm_sum;
      
      //occupancy smoothening
      float occ_sf = SMOOTH_FACTOR;
      float occ_sf_o = 1 - occ_sf;
      norm_sum = 0;
      for (int m = 0; m < 2; m++) {
        float sum = 0;
        for (int n = 0; n < 2; n++) {
          if(m == n)
            sum += occupancy[n]*occ_sf;
          else
            sum += occupancy[n]*occ_sf_o;
        }
        norm_sum += sum;
        upd_occupancy.push_back(sum);
      }
      for(int m = 0; m < 2; m++)
        upd_occupancy[m] /= norm_sum;

      // update the temp tree
      it->setLogOdds(octomap::logodds(upd_occupancy[0]));
      it->setSemantics(upd_labels);
    }

 
    //------------------------- delete the voxel that has been updated in orginal tree----------------------------------//
    
    for (int i=0; i<(int)cloud->size(); i++) {
      const point3d& query = (*cloud)[i];
      SemanticOcTreeNode* n = tree.search(query);
      if(!n)
        continue;
      else
        tree.deleteNode(query);
    }

 
    //------------------------- update the original tree----------------------------------//
    
    for (SemanticOcTree::leaf_iterator it = temp_tree->begin_leafs(),
        end = temp_tree->end_leafs(); it != end; ++it)
    {
      point3d query = it.getCoordinate();
      SemanticOcTreeNode* n = tree.search(query);
      float ot = it->getLogOdds();
      SemanticOcTreeNode::Semantics st = it->getSemantics();
      
      if(!n) {
        SemanticOcTreeNode* new_node = tree.updateNode(query, ot);
        new_node->setSemantics(st);
      }
      else{
        n->setLogOdds(ot);
        n->setSemantics(st);
      }
    }
    delete temp_tree;
    delete cloud;
    
    //----------------------------------------------------end prediction----------------------------------//


    // traverse the whole tree, set color based on semantics to visualize
    for (SemanticOcTree::iterator it = tree.begin(); it != tree.end(); ++it) {
      if ( it->isSemanticsSet() ) {
        SemanticOcTreeNode::Semantics s = it->getSemantics();
        // Debug
        //print_query_info(point3d(0,0,0), &(*it)); 
        for (int i = 0; i < NUM_LABELS; i++) {
          if (s.label.size() && s.label[i] > 0.6) {
            it->setColor(color[i][0], color[i][1], color[i][2]);
          }
        }
      }
    }//end for
    outfile = std::string(argv[argn]) + "_global_tree.ot";
    tree.write(outfile);
  } // END THE BIG FOR

  cout << "Test done." << endl;
  exit(0);
}
