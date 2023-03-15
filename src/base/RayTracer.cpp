#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>
#include <algorithm>
#include <stack>
#include "rtlib.hpp"
#define FLOAT_MAX 3.40282e+38
#define FLOAT_MIN -3.40282e+38

// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);


namespace FW
{


	Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
	{

		// YOUR CODE HERE (R3):
		// Get texel indices of texel nearest to the uv vector. Used in texturing.
		// UV coordinates range from negative to positive infinity. First map them
		// to a range between 0 and 1 in order to support tiling textures, then
		// scale the coordinates by image resolution and find the nearest pixel.

		Vec2f res;
		res.x = std::abs((uv.x - std::floor(uv.x))) * size.x;
		res.y = std::abs((uv.y - std::floor(uv.y))) * size.y;


		return res;
	}

	Mat3f formBasis(const Vec3f& n) {
		// YOUR CODE HERE (R4):

		//q is not parallel to n
		Vec3f q = n;
		std::vector<float> qabs = { abs(q[0]), abs(q[1]), abs(q[2]) };

		auto it = std::min_element(qabs.begin(), qabs.end());
		int index = std::distance(qabs.begin(), it);
		q[index] = 1.0f;

		Vec3f t = cross(q, n).normalized();
		Vec3f b = cross(n, t).normalized();

		Mat3f res;
		res.setCol(0, t);
		res.setCol(1, b);
		res.setCol(2, n);
		return res;
	}


	String RayTracer::computeMD5(const std::vector<Vec3f>& vertices)
	{
		unsigned char digest[16];
		MD5Buffer((void*)&vertices[0], sizeof(Vec3f) * vertices.size(), (unsigned int*)digest);

		// turn into string
		char ad[33];
		for (int i = 0; i < 16; ++i)
			::sprintf(ad + i * 2, "%02x", digest[i]);
		ad[32] = 0;

		return FW::String(ad);
	}


	// --------------------------------------------------------------------------


	RayTracer::RayTracer()
	{
	}

	RayTracer::~RayTracer()
	{
	}


	void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
	{
		std::ifstream ifs(filename, std::ios::binary);
		m_bvh = Bvh(ifs);

		m_triangles = &triangles;
	}

	void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
		(void)triangles; // Not used.

		std::ofstream ofs(filename, std::ios::binary);
		m_bvh.save(ofs);
	}

	void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
		// YOUR CODE HERE (R1):
		// This is where you should construct your BVH.
		printf("Split Mode: %d \n", (int)splitMode);
		m_triangles = &triangles;
		m_bvh.construct(triangles, splitMode);
	}
	RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const {
		++m_rayCount;

		// YOUR CODE HERE (R1):
		// This is where you traverse the tree you built! It's probably easiest
		// to introduce another function above that does the actual traversal, and
		// use this function only to begin the recursion by calling the traversal
		// function with the given ray and your root node. You can also use this
		// function to do one-off things per ray like finding the elementwise
		// reciprocal of the ray direction.

#pragma region Naive Loop
		//float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
		//int closest_i = -1;
		//RaycastResult castresult;
		//for (int i = 0; i < (int)m_triangles->size(); ++i)
		//{
		//	float t, u, v;
		//	if ((*m_triangles)[i].intersect_woop(orig, dir, t, u, v))
		//	{
		//		if (t > 0.0f && t < closest_t)
		//		{
		//			closest_i = i;
		//			closest_t = t;
		//			closest_u = u;
		//			closest_v = v;
		//		}
		//	}
		//}

		//if (closest_i != -1)
		//	castresult = RaycastResult(&(*m_triangles)[closest_i], closest_t, closest_u, closest_v, orig + closest_t * dir, orig, dir);

		//return castresult;
#pragma endregion

		Vec3f reci_dir_unit = 1.0f / dir;

		return raycastBvhStack(orig, dir, reci_dir_unit, m_bvh.root());
		//return raycastBvhIterator(orig, dir, reci_dir_unit, m_bvh.root());
	}

	bool RayTracer::CheckIntersection(const Vec3f& orig, const Vec3f& dir, const Vec3f& reci_dir, const BvhNode& node, float& t_hit) const{
		//Check Intersections with AABB
		//t1: ray inject hit.  t2: ray leave hit.
		//Compute t1,t2 for each dimention, and make sure t1[i] < t2[i];
		//Ray can inject from every direciton, thus we can not ensure t1 is computed from bb.min
		Vec3f t1, t2;
		for (int i = 0; i < 3; ++i) {
			t1[i] = (node.bb.min[i] - orig[i]) * reci_dir[i];
			t2[i] = (node.bb.max[i] - orig[i]) * reci_dir[i];
			if (t1[i] > t2[i])
				std::swap(t1[i], t2[i]);
		}
		
		FW::F32 tstart = t1.max(); 
		FW::F32 tend = t2.min(); 

		// A bug that I had before:
		// t_hit should be [0, +¡Þ)£¬ rather than [0,1].
		// Because when checking a large bouding box with a short ray, the ray orig in inside the bb,
		// the tend might be large, however, there still probably exists intersection because smaller bounding boxs
		// are inside this bouding box.
		if (tstart > tend) return false;
		if (tend < 0) return false;
		if (tstart > 0 && tstart < 1) {
			t_hit = tstart;
			return true;
		}
		else if(tstart < 0){
			t_hit = tend;
			return true;
		}
	}
	RaycastResult RayTracer::raycastBvhStack(const Vec3f& orig, const Vec3f& dir, const Vec3f& reci_dir, const BvhNode& node) const {
		std::stack<const BvhNode*> nodeStack;
		nodeStack.push(&node); //Add Root
		RaycastResult castresult;
		float best_t = FLOAT_MAX;
		while (!nodeStack.empty()) {
			const BvhNode* currectNode = nodeStack.top();
			nodeStack.pop();

			if (!currectNode->hasChildren()) {
				//Do traversal in a leaf node
				//t range [0,1]
				float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
				int closest_i = -1;

				// Naive loop over all triangles; this will give you the correct results,
				// but is terribly slow when ran for all triangles for each ray. Try it.
				for (int i = currectNode->startPrim; i < currectNode->endPrim; ++i)
				{
					float t, u, v;
					if ((*m_triangles)[m_bvh.getIndex(i)].intersect_woop(orig, dir, t, u, v))
					{
						if (t > 0.0f && t < closest_t)
						{
							closest_i = i;
							closest_t = t;
							closest_u = u;
							closest_v = v;
						}
					}
				}
				if (closest_i != -1 && closest_t < best_t) {
					RaycastResult temp = RaycastResult(&(*m_triangles)[m_bvh.getIndex(closest_i)], closest_t, closest_u, closest_v, orig + closest_t * dir, orig, dir);
					castresult = temp;
					best_t = closest_t;
				}
				continue;
			}
			float t_hit = FLOAT_MAX;
			if (CheckIntersection(orig, dir, reci_dir, *currectNode, t_hit) && t_hit < best_t) {
				if ((currectNode->left->bb.center() - orig).length() < (currectNode->right->bb.center() - orig).length()) {
					nodeStack.push(currectNode->right.get());
					nodeStack.push(currectNode->left.get());
				}
				else {
					nodeStack.push(currectNode->left.get());
					nodeStack.push(currectNode->right.get());
				}
				continue;
			}
			else if (currectNode->bb.contains(orig)) {
				if ((currectNode->left->bb.center() - orig).length() < (currectNode->right->bb.center() - orig).length()) {
					nodeStack.push(currectNode->right.get());
					nodeStack.push(currectNode->left.get());
				}
				else {
					nodeStack.push(currectNode->left.get());
					nodeStack.push(currectNode->right.get());
				}
				continue;
			}
		}
		return castresult;
	}
	RaycastResult RayTracer::raycastBvhIterator(const Vec3f& orig, const Vec3f& dir, const Vec3f& reci_dir, const BvhNode& node) const {
		RaycastResult castresult;
		if (!node.hasChildren()) {
			//Do traversal in a leaf node
			//t range [0,1]
			float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
			int closest_i = -1;

			// Naive loop over all triangles; this will give you the correct results,
			// but is terribly slow when ran for all triangles for each ray. Try it.
			for (int i = node.startPrim; i < node.endPrim; ++i)
			{
				float t, u, v;
				if ((*m_triangles)[m_bvh.getIndex(i)].intersect_woop(orig, dir, t, u, v))
				{
					if (t > 0.0f && t < closest_t)
					{
						closest_i = i;
						closest_t = t;
						closest_u = u;
						closest_v = v;
					}
				}
			}
			if (closest_i != -1)
				castresult = RaycastResult(&(*m_triangles)[m_bvh.getIndex(closest_i)], closest_t, closest_u, closest_v, orig + closest_t * dir, orig, dir);

			return castresult;
		}


		//Has intersections, Go deeper
		float t_left_hit = FLOAT_MAX, t_right_hit = FLOAT_MAX;
		bool hitLeft = false, hitRight = false;
		hitLeft = CheckIntersection(orig, dir, reci_dir, *node.left, t_left_hit);
		hitRight = CheckIntersection(orig, dir, reci_dir, *node.right, t_right_hit);
		if (hitLeft && !hitRight)
			//Only Hit Left
			return raycastBvhIterator(orig, dir, reci_dir, *node.left);
		if (!hitLeft && hitRight)
			//Only Hit Right
			return raycastBvhIterator(orig, dir, reci_dir, *node.right);
		if (!hitLeft && !hitRight)
			//Hit Nothing
			return castresult;

		//If both hit, compare the closest t
		RaycastResult resultL, resultR;
		//Otherwise, find the exact hit of both and compare.
		//Traverse the closest node firstly
		if (t_left_hit < t_right_hit) {
			resultL = raycastBvhIterator(orig, dir, reci_dir, *node.left);
			if (resultL.t < t_right_hit && !node.right->bb.contains(orig)) return resultL;
			resultR = raycastBvhIterator(orig, dir, reci_dir, *node.right);
			return resultL.t < resultR.t ? resultL : resultR;
		}
		else {
			resultR = raycastBvhIterator(orig, dir, reci_dir, *node.right);
			if (resultR.t < t_left_hit && !node.left->bb.contains(orig)) return resultR;
			resultL = raycastBvhIterator(orig, dir, reci_dir, *node.left);
			return resultL.t < resultR.t ? resultL : resultR;
		}

	}


} // namespace FW