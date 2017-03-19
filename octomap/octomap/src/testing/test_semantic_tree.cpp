#include <iostream>
#include <octomap/octomap.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"

using namespace std;
using namespace octomap;


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

  double res = 0.05;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
  SemanticOcTree tree (res);
  // insert some measurements of occupied cells
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f+0.01f, (float) y*0.05f+0.01f, (float) z*0.05f+0.01f);
        SemanticOcTreeNode* n = tree.updateNode(endpoint, true); 
        n->setColor(z*5+100,x*5+100,y*5+100); 
        std::vector<float> labels;
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        n->setSemantics(labels);
      }
    }
  }

  // insert some measurements of free cells
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
        SemanticOcTreeNode* n = tree.updateNode(endpoint, false); 
        n->setColor(255,255,0); // set color to yellow
        std::vector<float> labels;
        labels.push_back(0.5);
        labels.push_back(0.3);
        labels.push_back(0.2);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        labels.push_back(0.7);
        labels.push_back(0.2);
        labels.push_back(0.1);
        n->setSemantics(labels);
      }
    }
  }

  // set inner node colors
  tree.updateInnerOccupancy();
  // should already be pruned
  EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
  const size_t initialSize = tree.size();
  EXPECT_EQ(initialSize, 1034);
  //tree.prune();
  EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
  EXPECT_EQ(initialSize, tree.size());

  cout << endl;

  std::cout << "\nWriting to / from file\n===============================\n";
  std::string filename ("simple_semantic_tree.ot");
  std::cout << "Writing semantic tree to " << filename << std::endl;
  // write color tree
  EXPECT_TRUE(tree.write(filename));


  // read tree file
  cout << "Reading semantic tree from "<< filename <<"\n";
  AbstractOcTree* read_tree = AbstractOcTree::read(filename);
  EXPECT_TRUE(read_tree);
  EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
  EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
  EXPECT_EQ(read_tree->size(), tree.size());
  SemanticOcTree* read_semantic_tree = dynamic_cast<SemanticOcTree*>(read_tree);
  EXPECT_TRUE(read_semantic_tree);

  EXPECT_TRUE(tree == *read_semantic_tree);

 
  {
    cout << "Performing some queries:" << endl;
    // TODO: some more meaningful tests
    point3d query (0., 0., 0.);
    SemanticOcTreeNode* result = tree.search (query);
    SemanticOcTreeNode* result2 = read_semantic_tree->search (query);
    std::cout << "READ: ";
    print_query_info(query, result);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_TRUE(result);
    EXPECT_TRUE(result2);
    EXPECT_EQ(result->getColor(), result2->getColor());
    EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());
    
    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);
    result2 = read_semantic_tree->search (query);
    print_query_info(query, result);
    std::cout << "READ: ";
    print_query_info(query, result);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_TRUE(result);
    EXPECT_TRUE(result2);
    EXPECT_EQ(result->getColor(), result2->getColor());
    EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());
    
    query = point3d(1.,1.,1.);
    result = tree.search (query);
    result2 = read_semantic_tree->search (query);
    print_query_info(query, result);
    std::cout << "READ: ";
    print_query_info(query, result);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_FALSE(result);
    EXPECT_FALSE(result2);
  }

  delete read_tree;
  read_tree = NULL;
  
  {
    std::cout << "\nPruning / expansion\n===============================\n";
    EXPECT_EQ(initialSize, tree.size());
    EXPECT_EQ(initialSize, tree.calcNumNodes());
    std::cout << "Initial size: " << tree.size() << std::endl;
    
    // tree should already be pruned during insertion:
    tree.prune();
    EXPECT_EQ(initialSize, tree.size());
    EXPECT_EQ(initialSize, tree.calcNumNodes());
        
    tree.expand();
    std::cout << "Size after expansion: " << tree.size() << std::endl;
    EXPECT_EQ(tree.size(), tree.calcNumNodes());
    
    // prune again, should be same as initial size
    
    tree.prune();
    EXPECT_EQ(initialSize, tree.size());
    EXPECT_EQ(initialSize, tree.calcNumNodes());
        
  }
  
  // delete / create some nodes
  {
    std::cout << "\nCreating / deleting nodes\n===============================\n";
    size_t initialSize = tree.size();
    EXPECT_EQ(initialSize, tree.calcNumNodes());
    std::cout << "Initial size: " << initialSize << std::endl;
    
    point3d newCoord(-2.0, -2.0, -2.0);
    SemanticOcTreeNode* newNode = tree.updateNode(newCoord, true);
    newNode->setColor(255,0,0);
    EXPECT_TRUE(newNode != NULL);
    
    const size_t insertedSize = tree.size();
    std::cout << "Size after one insertion: " << insertedSize << std::endl;
    EXPECT_EQ(insertedSize, initialSize+6);
    EXPECT_EQ(insertedSize, tree.calcNumNodes());
    
    // find parent of newly inserted node:
    SemanticOcTreeNode* parentNode = tree.search(newCoord, tree.getTreeDepth() -1);
    EXPECT_TRUE(parentNode);
    EXPECT_TRUE(tree.nodeHasChildren(parentNode));
    
    // only one child exists:
    EXPECT_TRUE(tree.nodeChildExists(parentNode, 0));
    for (size_t i = 1; i < 8; ++i){
      EXPECT_FALSE(tree.nodeChildExists(parentNode, i));
    }
    
    tree.deleteNodeChild(parentNode, 0);
    EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
    EXPECT_EQ(tree.size(), insertedSize - 1);
    
    tree.prune();
    EXPECT_EQ(tree.size(), tree.calcNumNodes());
    EXPECT_EQ(tree.size(), insertedSize - 1);
    
    tree.expandNode(parentNode);
    EXPECT_EQ(tree.size(), tree.calcNumNodes());
    EXPECT_EQ(tree.size(), insertedSize + 7);
    
    EXPECT_TRUE(tree.pruneNode(parentNode));
    EXPECT_EQ(tree.size(), tree.calcNumNodes());
    EXPECT_EQ(tree.size(), insertedSize - 1);
    
    EXPECT_TRUE(tree.write("simple_semantic_tree_ed.ot"));
    
  }

  return 0;
}
