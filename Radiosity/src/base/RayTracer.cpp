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
extern "C" void MD5Buffer( void* buffer, size_t bufLen, unsigned int* pDigest );


namespace FW
{


Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
{

	// Convert texture coordinate to pixel index as you did in assignment 1.
    Vec2f res;
    res.x = std::abs((uv.x - std::floor(uv.x))) * size.x;
    res.y = std::abs((uv.y - std::floor(uv.y))) * size.y;


    return res;
}

Mat3f formBasis(const Vec3f& n) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
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


String RayTracer::computeMD5( const std::vector<Vec3f>& vertices )
{
    unsigned char digest[16];
    MD5Buffer( (void*)&vertices[0], sizeof(Vec3f)*vertices.size(), (unsigned int*)digest );

    // turn into string
    char ad[33];
    for ( int i = 0; i < 16; ++i )
        ::sprintf( ad+i*2, "%02x", digest[i] );
    ad[32] = 0;

    return FW::String( ad );
}


// --------------------------------------------------------------------------


RayTracer::RayTracer()
{
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    // After that you can remove the following line.
    //m_rt.reset(new rtlib::RayTracer);
}

RayTracer::~RayTracer()
{
}


void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
{
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
	//m_rt->loadHierarchy(filename, triangles);

    std::ifstream ifs(filename, std::ios::binary);
    m_bvh = Bvh(ifs);

	m_triangles = &triangles;
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    //m_rt->saveHierarchy(filename, triangles);

    std::ofstream ofs(filename, std::ios::binary);
    m_bvh.save(ofs);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
	//m_rt->constructHierarchy(triangles, splitMode);
	//m_triangles = &triangles;

    printf("Split Mode: %d \n", (int)splitMode);
    m_triangles = &triangles;
    m_bvh.construct(triangles, splitMode);
}


RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const {
	++m_rayCount;

    // YOUR CODE HERE (R1):
    // Integrate your implementation here.

    Vec3f reci_dir = 1.0f / dir;
    return raycastBvhIterator(orig, dir, reci_dir, m_bvh.root());
}
bool RayTracer::CheckIntersection(const Vec3f& orig, const Vec3f& dir, const Vec3f& reci_dir, const BvhNode& node, float& t_hit) const {
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
	else if (tstart < 0) {
		t_hit = tend;
		return true;
	}
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
		//If the actual hit from the left is closer than hit of right BB, 
		//and the right BB does not contain ray origin(otherwise it may have a closer triangle)
		//then we don't need to go to right tree
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