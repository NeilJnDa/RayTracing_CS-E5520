
#include "Bvh.hpp"
#include "filesaves.hpp"
#include <algorithm>

#define FLOAT_MAX 3.40282e+38
#define FLOAT_MIN -3.40282e+38
#define BUCKETS 8
#define N 50  //Max primitives in a bucket
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
	void printNodeInfo(std::unique_ptr<BvhNode>& node, const std::vector<RTTriangle>& triangles, SplitMode splitMode, std::vector<uint32_t>& indices_, int depth) {
		if(!node->hasChildren())
			printf("____________Leaf: depth: %d \n", depth);
		else
			printf("____________Node: depth: %d \n", depth);

		printf("Prims : %d \n", node->endPrim - node->startPrim);
		FW::printVec3f("BB min", node->bb.min);
		FW::printVec3f("BB max", node->bb.max);

		printf("node start: %d \n", node->startPrim);
		printf("node end: %d \n", node->endPrim);
		Vec3f averageCenter;
		for (int i = node->startPrim; i < node->endPrim; ++i) {
			//FW::printVec3f("Triangle V1", triangles[indices_[i]].m_vertices[0].p);
			//FW::printVec3f("Triangle V2", triangles[indices_[i]].m_vertices[1].p);
			//FW::printVec3f("Triangle V3", triangles[indices_[i]].m_vertices[2].p);
			//FW::printVec3f("Triangle Center", triangles[indices_[i]].centroid());
			averageCenter += triangles[indices_[i]].centroid();
		}
		FW::printVec3f("AverageCenter", averageCenter / (node->endPrim - node->startPrim));
		printf("\n");
	}
	void constructIterator(std::unique_ptr<BvhNode>& node, const std::vector<RTTriangle>& triangles, SplitMode splitMode, std::vector<uint32_t>& indices_, int depth) {
		depth++;
		node->bb.min = Vec3f(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX);
		node->bb.max = Vec3f(FLOAT_MIN, FLOAT_MIN, FLOAT_MIN);

		
		//Find partition according to BB of triangles centroids. This could prevent from a too large triangle making it hard to partition
		//tcBB: Triangles Centroids Bounding Box
		AABB tcBB(Vec3f(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX), Vec3f(FLOAT_MIN, FLOAT_MIN, FLOAT_MIN));

		for (int i = node->startPrim; i < node->endPrim; ++i) {
			node->bb.max = FW::max(node->bb.max, triangles[indices_[i]].max());
			node->bb.min = FW::min(node->bb.min, triangles[indices_[i]].min());
			tcBB.max = FW::max(tcBB.max, triangles[indices_[i]].centroid());
			tcBB.min = FW::min(tcBB.min, triangles[indices_[i]].centroid());
		}

		//NOTE: if all triangles are on a surface, one axis can be close to 0. 
		//Thus when check intersections, the check should be <= rather than <
		node->left = nullptr;
		node->right = nullptr;

		//  If less than certain number of Prims, this is a leaf node.
		if (node->endPrim - node->startPrim < N) {
			//printNodeInfo(node, triangles, splitMode, indices_, depth);
			return;
		}

		//  Select Partition
		int partitionIndex = -1; //The first one that is in the second part;

#pragma region Spatial Median
		if (splitMode == SplitMode::SplitMode_SpatialMedian) {
			FW::Vec3f diagonal = node->bb.max - node->bb.min;
			//Spatial Partition doesn't work well
			FW::Vec3f center = node->bb.min + diagonal / 2.0f;
			if (diagonal.x >= diagonal.y && diagonal.x >= diagonal.z) {
				//  Sort by X
				//FW::printf("Sort by X: %d \n", center.x);
				auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
					[&triangles, &center](int a) {return triangles[a].centroid().x < center.x; });
				partitionIndex = std::distance(indices_.begin(), it);
			}
			else if (diagonal.y >= diagonal.z && diagonal.y >= diagonal.x) {
				//  Sort by Y
				//FW::printf("Sort by Y: %d \n", center.y);
				auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
					[&triangles, &center](int a) {return triangles[a].centroid().y < center.y; });
				partitionIndex = std::distance(indices_.begin(), it);
			}
			else {
				//  Sort by Z
				//FW::printf("Sort by Z: %d \n", center.z);
				auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
					[&triangles, &center](int a) {return triangles[a].centroid().z < center.z; });
				partitionIndex = std::distance(indices_.begin(), it);
			}
		}


#pragma endregion
#pragma region Object Median Partition
		if (splitMode == SplitMode::SplitMode_ObjectMedian) {
			FW::Vec3f diagonal = node->bb.max - node->bb.min;
			partitionIndex = node->startPrim + (node->endPrim - node->startPrim) / 2;
			if (diagonal.x >= diagonal.y && diagonal.x >= diagonal.z) {
				//  Sort by X
				std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
					[&triangles](int a, int b) {return triangles[a].centroid().x < triangles[b].centroid().x; }
				);
			}
			else if (diagonal.y >= diagonal.z && diagonal.y >= diagonal.x) {
				//  Sort by Y
				std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
					[&triangles](int a, int b) {return triangles[a].centroid().y < triangles[b].centroid().y; }
				);
			}
			else {
				//  Sort by Z
				std::nth_element(indices_.begin() + node->startPrim, indices_.begin() + partitionIndex, indices_.begin() + node->endPrim,
					[&triangles](int a, int b) {return triangles[a].centroid().z < triangles[b].centroid().z; }
				);
			}
		}
#pragma endregion
#pragma region SAH Surface Area Heuristic Partition
		if (splitMode == SplitMode::SplitMode_Sah) {
			FW::Vec3f diagonal = node->bb.max - node->bb.min;
			FW::Vec3f diagonal_tc = tcBB.max - tcBB.min;
			float minSAH = FLOAT_MAX;
			int minSAHAxis = 0;
			float minPlane = 0;

			//Check Partition for each axis
			for (int axis = 0; axis < 3; ++axis) {
				float interval = diagonal_tc[axis] / BUCKETS;

				if (diagonal_tc[axis] < 1e-3) {
					//To small scale, do not split
					//printf("Too small Splitting! axis: %f, diagonal_tc[axis]: %f \n", axis, diagonal_tc[axis]);
					continue;
				}

				//For each axis, do several times of checking with uniformly distributed interval.
				//Dynamic Programming: Store the number of triangles in the left node outside the loop
				//Since the bucket starts from tcBB.min rather than node->bb.min. There are two more space before the first bucket and after the last.
				std::vector<int> triangles_count(BUCKETS + 2, 0);
				std::for_each(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim, 
					[&](int a) {
						int bucket = (triangles[a].centroid()[axis] - tcBB.min[axis]) / interval;
						triangles_count[bucket + 1]++;
					});
				float plane = tcBB.min[axis];
				float leftTri = 0;
				for (int i = 0; i <= BUCKETS + 1; ++i) {

					leftTri += triangles_count[i];
					Vec3f leftBBMax = tcBB.max;
					leftBBMax[axis] = plane;
					AABB leftBB(tcBB.min, leftBBMax);

					Vec3f rightBBmin = tcBB.min;
					rightBBmin[axis] = plane;
					AABB rightBB(rightBBmin, tcBB.max);

					//Calculate: SAH = AL * NL + AR * NR.  This holds when if we stop splitting ealier when number of trianglers is smaller than a value
					float SAH = leftTri * leftBB.area() + (node->endPrim - node->startPrim - leftTri) * rightBB.area();

					//Update Min SAH
					if (SAH < minSAH) {

						minSAH = SAH;
						minSAHAxis = axis;
						minPlane = plane;
					}
					plane += interval;
				}

			}

			auto it = std::partition(indices_.begin() + node->startPrim, indices_.begin() + node->endPrim,
				[&triangles, &minSAHAxis, &minPlane](int a) {return triangles[a].centroid()[minSAHAxis] < minPlane; });
			partitionIndex = std::distance(indices_.begin(), it);
		}
		//
#pragma endregion
		if (partitionIndex == node->startPrim || partitionIndex == node->endPrim) {
			//printNodeInfo(node, triangles, splitMode, indices_, depth);
			return;
		}
		node->left = std::make_unique<BvhNode>(node->startPrim, partitionIndex);
		constructIterator(node->left, triangles, splitMode, indices_, depth);		
		node->right = std::make_unique<BvhNode>(partitionIndex, node->endPrim);
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
