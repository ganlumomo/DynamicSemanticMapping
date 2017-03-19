/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/SemanticOcTree.h>

namespace octomap {


  // node implementation  --------------------------------------
  std::ostream& SemanticOcTreeNode::writeData(std::ostream &s) const {
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &color, sizeof(Color)); // color
    
    return s;
  }

  std::istream& SemanticOcTreeNode::readData(std::istream &s) {        
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &color, sizeof(Color)); // color
    
    return s;
  }

  SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor() const {
    int mr = 0;
    int mg = 0;
    int mb = 0;
    int c = 0;
    
    if (children != NULL){
      for (int i=0; i<8; i++) {
        SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);
        
        if (child != NULL && child->isColorSet()) {
          mr += child->getColor().r;
          mg += child->getColor().g;
          mb += child->getColor().b;
          ++c;
        }
      }
    }
    
    if (c > 0) {
      mr /= c;
      mg /= c;
      mb /= c;
      return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
    }
    else { // no child had a color other than white
      return Color(255, 255, 255);
    }
  }


  SemanticOcTreeNode::Semantics SemanticOcTreeNode::getAverageChildSemantics() const {
    std::vector<float> mlabel;
    int c = 0;

    if (children != NULL){
      for (int i=0; i<8; i++) {
        SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);
        if (child != NULL && child->isSemanticsSet()) {
          std::vector<float> clabel = child->getSemantics().label;
          if (mlabel.empty())
            mlabel.reserve(clabel.size());
          else if (mlabel.size() < clabel.size())
            mlabel.resize(clabel.size());

          for(int l=0; l<(int)clabel.size(); l++) {
            mlabel[l] += clabel[l];
          }
          ++c;
        }
      }
    }
    
    if (c > 0) {
      for (int l=0; l<(int)mlabel.size(); l++) {
        mlabel[l] /= c;
      }
      return Semantics(mlabel);
    }
    else { // no child had a semantics other than empty
      return Semantics();
    }
  }


  void SemanticOcTreeNode::updateColorChildren() {      
    color = getAverageChildColor();
  }
  
  void SemanticOcTreeNode::updateSemanticsChildren() {      
    semantics = getAverageChildSemantics();
  }


  // tree implementation  --------------------------------------
  SemanticOcTree::SemanticOcTree(double resolution)
  : OccupancyOcTreeBase<SemanticOcTreeNode>(resolution) {
    semanticOcTreeMemberInit.ensureLinking();
  };

 
  void SemanticOcTree::averageNodeColor(SemanticOcTreeNode* n,
                                        uint8_t r,
                                        uint8_t g,
                                        uint8_t b) {
    if (n != 0) {
      if (n->isColorSet()) {
        SemanticOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
      }
      else {
        n->setColor(r, g, b);
      }
    }
  }

  void SemanticOcTree::averageNodeSemantics(SemanticOcTreeNode* n,
                                            SemanticOcTreeNode::Semantics s) {
    if (n != 0) {
      if (n->isSemanticsSet()) {
        SemanticOcTreeNode::Semantics prev_semantics = n->getSemantics();
        if (prev_semantics.label.size() < s.label.size())
            prev_semantics.label.resize(s.label.size());
       
        for (int i=0; i<(int)s.label.size(); i++) {
          prev_semantics.label[i] += s.label[i];
          prev_semantics.label[i] /= 2;
        }
        n->setSemantics(prev_semantics);
      }
      else {
        n->setSemantics(s);
      }
    }
  }


  void SemanticOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void SemanticOcTree::updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node)){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (nodeChildExists(node, i)) {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateColorChildren();
      node->updateSemanticsChildren();
    }
  }
  
  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b <<
                  ')';
  }

  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Semantics const& s) {
    for (unsigned i=0; i<s.label.size(); i++) {
      out << s.label[i] << ' ';
    }
    return out;
  }


  SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit;

} // end namespace

