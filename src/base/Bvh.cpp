
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

			for (size_t i = 0; i < size; ++i) {
				uint32_t idx;
				loader(idx);
				indices_.push_back(idx);
			}
		}

		// Load the rest.
		rootNode_.reset(new BvhNode(loader));
	}

	void constructIterator(std::unique_ptr<BvhNode>& node, const std::vector<RTTriangle>& triangles, SplitMode splitMode, std::vector<uint32_t>& indices_, int depth) {
		depth++;
		node->bb.max = Vec3f(std::numeric_limits<FW::F32>::min(), std::numeric_limits<FW::F32>::min(), std::numeric_limits<FW::F32>::min());
		node->bb.min = Vec3f(std::numeric_limits<FW::F32>::max(), std::numeric_limits<FW::F32>::max(), std::numeric_limits<FW::F32>::max());

		//printf("node start: %d \n", node->startPrim);
		//printf("node end: %d \n", node->endPrim);
		for (int i = node->startPrim; i < node->endPrim; ++i) {
			node->bb.max = FW::max(node->bb.max, triangles[indices_[i]].max());
			node->bb.min = FW::min(node->bb.min, triangles[indices_[i]].min());
		}
		node->left = nullptr;
		node->right = nullptr;

		//  If less than certain number of Prims, this is a leaf node.
		if (node->endPrim - node->startPrim < 50) {

			//printf("____________Leaf: depth: %d \n", depth);
			//printf("node start: %d \n", node->startPrim);
			//printf("node end: %d \n", node->endPrim);
			//printf("Prims : %d \n", node->endPrim - node->startPrim);
			//for (int i = node->startPrim; i < node->endPrim; ++i) {
			//	FW::printVec3f(triangles[indices_[i]].centroid());
			//}
			return;
		}

		//  Select Partition
		int partitionIndex = -1; //The first one that is in the second part;

#pragma region Spatial Median
	//FW::Vec3f diagonal = node->bb.max - node->bb.min;
	////Spatial Partition doesn't work well
	//FW::Vec3f center = node->bb.min + diagonal / 2.0f;

	////FW::printVec3f("min", node->bb.min);
	////FW::printVec3f("max", node->bb.max);
	////FW::printVec3f("diag", diagonal);
	////FW::printVec3f("center", center);
	//if (diagonal.x >= diagonal.y && diagonal.x >= diagonal.z) {
	//    //  Sort by X
	//    //FW::printf("Sort by X: %d \n", center.x);
	//    auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
	//        [&triangles, &center](int a) {return triangles[a].centroid().x < center.x; });
	//    partitionIndex = std::distance(indices_.begin(), it);
	//}
	//else if (diagonal.y >= diagonal.z && diagonal.y >= diagonal.x) {
	//    //  Sort by Y
	//    //FW::printf("Sort by Y: %d \n", center.y);
	//    auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
	//        [&triangles, &center](int a) {return triangles[a].centroid().y < center.y; });
	//    partitionIndex = std::distance(indices_.begin(), it);
	//}
	//else {
	//    //  Sort by Z
	//    //FW::printf("Sort by Z: %d \n", center.z);
	//    auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
	//        [&triangles, &center](int a) {return triangles[a].centroid().z < center.z; });
	//    partitionIndex = std::distance(indices_.begin(), it);
	//}

#pragma endregion
#pragma region Object Median Partition
		//FW::Vec3f diagonal = node->bb.max - node->bb.min;
		//partitionIndex = node->startPrim + (node->endPrim - node->startPrim) / 2;
		//if (diagonal.x >= diagonal.y && diagonal.x >= diagonal.z) {
		//	//  Sort by X
		//	std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
		//		[&triangles](int a, int b) {return triangles[a].centroid().x < triangles[b].centroid().x; }
		//	);
		//}
		//else if (diagonal.y >= diagonal.z && diagonal.y >= diagonal.x) {
		//	//  Sort by Y
		//	std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
		//		[&triangles](int a, int b) {return triangles[a].centroid().y < triangles[b].centroid().y; }
		//	);
		//}
		//else {
		//	//  Sort by Z
		//	std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
		//		[&triangles](int a, int b) {return triangles[a].centroid().z < triangles[b].centroid().z; }
		//	);
		//}
#pragma endregion
#pragma region SAH Surface Area Heuristic Partition
		FW::Vec3f diagonal = node->bb.max - node->bb.min;
		const float interval = 0.02f;
		float minSAH = std::numeric_limits<float>::max();
		int minSAHAxis = 0;
		float minPlane = 0;

		//Check Partition for each axis
		for (int axis = 0; axis < 3; ++axis) {

			float plane = node->bb.min[axis] + interval;
			//For each axis, do several times of checking with uniformly distributed interval.
			while (plane < node->bb.max[axis]) {
				
				//Count how many triangles will go to left child
				//for (int i = node->startPrim; i < node->endPrim; ++i) {
				//	if (triangles[indices_[i]].centroid()[axis] < plane) {
				//		//Will go to left
				//		leftTri++;
				//	}
				//}
				float leftTri = std::count_if(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim, 
					[&](int a) {return triangles[a].centroid()[axis] < plane; });
				Vec3f leftBBMax = node->bb.max;
				leftBBMax[axis] -= diagonal[axis] - plane;
				AABB leftBB(node->bb.min, leftBBMax);

				Vec3f rightBBmin = node->bb.min;
				rightBBmin[axis] += plane;
				AABB rightBB(rightBBmin, node->bb.max);

				//Calculate: SAH = AL * NL + AR * NR.  This holds when if we stop splitting ealier when number of trianglers is smaller than a value
				float SAH = leftTri * leftBB.area() + (node->endPrim - node->startPrim - leftTri) * rightBB.area();

				//Update Min SAH
				if (SAH < minSAH) {
					//printf("Current SAH: %f		", minSAH);
					//printf("New SAH: %f \n", SAH);

					minSAH = SAH;
					minSAHAxis = axis;
					minPlane = plane;
				}
				plane += interval;
			}
		}
		float SAH_NoSplitting = (node->endPrim - node->startPrim) * node->bb.area();
		if (SAH_NoSplitting < minSAH) {
			//No need to Split, this is a leaf Node
			printf("No need to Split: %f		", minSAH);
			printf("SAH_NoSplitting: %f \n", SAH_NoSplitting);
			return;
		}
		auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
			[&triangles, &minSAHAxis, &minPlane](int a) {return triangles[a].centroid()[minSAHAxis] < minPlane; });
		partitionIndex = std::distance(indices_.begin(), it);
		//
#pragma endregion



		if (partitionIndex == node->startPrim || partitionIndex == node->endPrim) {
			//All prims will go to either child, no need to partition
			//printf("All prims will go to either child. \n");
			//printf("____________Leaf: depth: %d \n", depth);
			//printf("node start: %d \n", node->startPrim);
			//printf("node end: %d \n", node->endPrim);
			//printf("Prims : %d \n", node->endPrim - node->startPrim);
			//for (int i = node->startPrim; i < node->endPrim; ++i) {
			//	FW::printVec3f(triangles[indices_[i]].centroid());
			//}
			return;
		}
		//FW::printf("LEFT: \n");
		//for (int i = node->startPrim; i < node->startPrim + partitionIndex; ++i) {
		//    printf("%d: index %d ", i, indices_[i]);
		//    FW::printVec3f(triangles[indices_[i] ].centroid());
		//}
		//FW::printf("RIGHT: \n");
		//for (int i = node->startPrim + partitionIndex; i < node->endPrim; ++i) {
		//    printf("%d: index %d ", i, indices_[i]);
		//    FW::printVec3f(triangles[indices_[i]].centroid());
		//}
		node->left = std::make_unique<BvhNode>(node->startPrim, partitionIndex);
		node->right = std::make_unique<BvhNode>(partitionIndex, node->endPrim);
		constructIterator(node->left, triangles, splitMode, indices_, depth);
		constructIterator(node->right, triangles, splitMode, indices_, depth);

	}
	void Bvh::construct(const std::vector<RTTriangle>& triangles, SplitMode splitMode) {
		this->mode_ = splitMode;
		this->indices_.resize(triangles.size());
		std::generate(indices_.begin(), indices_.end(), [n = 0]() mutable { return n++; });
		this->rootNode_ = std::make_unique<BvhNode>(0, triangles.size());
		//printf("triangles.size %d \n", triangles.size());
		//printf("Indice %d \n", getIndex(triangles.size() - 1));
		//printf("root range: %d to %d \n", root().startPrim, root().endPrim);

		constructIterator(this->rootNode_, triangles, splitMode, indices_, -1);

		BvhNode* current = &root();

	}
	void Bvh::save(std::ostream& os) {
		// Save file header.
		filesave(os, mode_);
		Statusbar nodeInfo("Saving nodes", 0);
		Saver saver(os, nodeInfo);

		// Save elements.
		{
			filesave(os, (size_t)indices_.size());

			for (auto& i : indices_) {
				saver(i);
			}
		}

		// Save the rest.
		rootNode_->save(saver);
	}

}
