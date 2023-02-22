
#include "Bvh.hpp"
#include "filesaves.hpp"
#include <limits>
#include <algorithm>


namespace FW {


Bvh::Bvh() { }


// reconstruct from a file
Bvh::Bvh(std::istream& is)
{
    // Load file header.
    fileload(is, mode_);
    Statusbar nodeInfo("Loading nodes", 0);
    Loader loader(is, nodeInfo);

    // Load elements.
    {
        size_t size;
        fileload(is, size);

        for(size_t i = 0; i < size; ++i) {
            uint32_t idx;
			loader(idx);
            indices_.push_back(idx);
        }
    }

    // Load the rest.
    rootNode_.reset(new BvhNode(loader));
}
void Bvh::construct(const std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    this->mode_ = splitMode;
    this->indices_.resize(triangles.size());
    std::generate(indices_.begin(), indices_.end(), [n = 0]() mutable { return n++; });
    this->rootNode_ = std::make_unique<BvhNode>(0, triangles.size());
    //printf("triangles.size %d \n", triangles.size());
    //printf("Indice %d \n", getIndex(triangles.size()-1));
    //printf("root range: %d to &d \n", root().startPrim, root().endPrim);
    constructIterator(this->rootNode_, triangles, splitMode);
}
void Bvh::constructIterator(std::unique_ptr<BvhNode> & node, const std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    node->bb.max = Vec3f(std::numeric_limits<FW::F32>::min(), std::numeric_limits<FW::F32>::min(), std::numeric_limits<FW::F32>::min());
    node->bb.min = Vec3f(std::numeric_limits<FW::F32>::max(), std::numeric_limits<FW::F32>::max(), std::numeric_limits<FW::F32>::max());

    for (int i = node->startPrim; i < node->endPrim; ++i) {
        node->bb.max = FW::max(node->bb.max, triangles[indices_[i]].max());
        node->bb.min = FW::min(node->bb.min, triangles[indices_[i]].min());
    }
    node->left = nullptr;
    node->right = nullptr;
    //FW::printVec3f(node->bb.min);
    //FW::printVec3f(node->bb.max);

    //  If less than certain number of Prims, this is a leaf node.
    if (node->endPrim - node->startPrim < 10) {
        return;
    }
    
    //  Select Partition Plane
    //  Spatial Median
    FW::Vec3f diagonal = node->bb.max - node->bb.min;
    if (diagonal.x >= diagonal.y && diagonal.x >= diagonal.z) {
        //  Sort by X

    }
    else if(diagonal.y >= diagonal.z && diagonal.y >= diagonal.x) {
        //  Sort by Y
    }
    else {
        //  Sort by Z

    }
}
void Bvh::save(std::ostream& os) {
    // Save file header.
    filesave(os, mode_);
    Statusbar nodeInfo("Saving nodes", 0);
    Saver saver(os, nodeInfo);

    // Save elements.
    {
        filesave(os, (size_t)indices_.size());

        for(auto& i : indices_) {
            saver(i);
        }
    }

    // Save the rest.
    rootNode_->save(saver);
}

}
