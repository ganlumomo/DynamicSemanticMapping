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

#ifndef OCTOMAP_SEMANTIC_OCTREE_H
#define OCTOMAP_SEMANTIC_OCTREE_H


#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {
  
  // forward declaraton for "friend"
  class SemanticOcTree;
 

  // node definition
  class SemanticOcTreeNode : public OcTreeNode {    
  public:
    friend class SemanticOcTree; // needs access to node children (inherited)
    
    class Color {
    public:
    Color() : r(255), g(255), b(255) {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b) 
      : r(_r), g(_g), b(_b) {}
      inline bool operator== (const Color &other) const {
        return (r==other.r && g==other.g && b==other.b);
      }
      inline bool operator!= (const Color &other) const {
        return (r!=other.r || g!=other.g || b!=other.b);
      }
      uint8_t r, g, b;
    };
 
    class Semantics {
    public:
    Semantics() : label() {}
    Semantics(std::vector<float> _label)
      : label(_label) {}
      // TODO-ganlu: inline bool operator== (const Semantics &other) const{}
      // TODO-ganlu: inline bool operator!= (const Semantics &other) const{}
      std::vector<float> label;
    };

  public:
    SemanticOcTreeNode() : OcTreeNode() {}

    SemanticOcTreeNode(const SemanticOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color), semantics(rhs.semantics){}

    // TODO-ganlu: bool operator==(const SemanticOcTreeNode& rhs) const{ }
    // TODO-ganlu: void copyData(const SemanticOcTreeNode& from) {}
        
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = c; }
    inline void  setColor(uint8_t r, uint8_t g, uint8_t b) {
      this->color = Color(r,g,b); 
    }
    Color& getColor() { return color; }
    
    inline Semantics getSemantics() const { return semantics; }
    inline void setSemantics(std::vector<float> s) {this->semantics = s; }
    Semantics& getSemantics() {return semantics; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const { 
      return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
    }
    
    inline bool isSemanticsSet() const { 
      return (semantics.label.size() > 0);
    }

    void updateColorChildren();
    void updateSemanticsChildren();

    SemanticOcTreeNode::Color getAverageChildColor() const;
    SemanticOcTreeNode::Semantics getAverageChildSemantics() const;  
    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;
    
  protected:
    Color color;
    Semantics semantics;
  };




  // tree definition
  class SemanticOcTree : public OccupancyOcTreeBase <SemanticOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    SemanticOcTree(double resolution);
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    SemanticOcTree* create() const {return new SemanticOcTree(resolution); }

    std::string getTreeType() const {return "ColorOcTree";}
    
    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    //void writeColorHistogram(std::string filename);
    
  protected:
    void updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           SemanticOcTree* tree = new SemanticOcTree(0.1);
           tree->clearKeyRays();
           AbstractOcTree::registerTreeType(tree);
         }

         /**
         * Dummy function to ensure that MSVC does not drop the
         * StaticMemberInitializer, causing this tree failing to register.
         * Needs to be called from the constructor of this octree.
         */
         void ensureLinking() {};
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer semanticOcTreeMemberInit;

  };

  //! user friendly output in format (r g b)
  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Color const& c);

  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Semantics const& s);

} // end namespace

#endif
