#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>

#include "rtlib.hpp"


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer( void* buffer, size_t bufLen, unsigned int* pDigest );


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
    return Mat3f();
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

    Vec3f reci_dir_unit = 1.0f / dir.normalized();
    return raycastBvhIterator(orig, dir, reci_dir_unit, m_bvh.root());
}
RaycastResult RayTracer::raycastBvhIterator(const Vec3f& orig, const Vec3f& dir, const Vec3f& reci_dir_unit, const BvhNode& node) const {
    RaycastResult castresult;
    if (!node.left && !node.right) {
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
        //
        //if (m_rayCount.load() == 30000) {
        //    FW::printVec3f("orig ", orig);
        //    FW::printVec3f(" dir ", dir);
        //    FW::printf(" dir length %f \n", dir.length());
        //    FW::printf("castResult: closest_t %f \n ", closest_t);

        //}
        return castresult;
    }
    
    
    //Check each dimention if parallel
    for (int i = 0; i < 3; ++i) {
        if(dir[i] == 0.0f && (orig[i] < node.bb.min[i] || orig[i] < node.bb.max[i]))
            return castresult;
    }


    //Check Intersections with AABB
    Vec3f t1 = (node.bb.min - orig) * reci_dir_unit;
    Vec3f t2 = (node.bb.max - orig) * reci_dir_unit;
    Vec3f tin = FW::min(t1, t2);
    Vec3f tout = FW::max(t1, t2);
    FW::F32 tstart = tin.max();
    FW::F32 tend = tout.min();

    //if (m_rayCount.load() == 300000) {
    //    printf("Current Node Contains: %d, %d \n ", node.startPrim, node.endPrim);
    //    FW::printVec3f("node.bb.min: ", node.bb.min);
    //    FW::printVec3f("node.bb.max: ", node.bb.max);
    //    FW::printVec3f("orig: ", orig);
    //    FW::printVec3f("dir: ", dir);
    //    FW::printVec3f("dir normalize: ", dir.normalized());
    //    FW::printVec3f("reci_dir: ", reci_dir);
    //    FW::printVec3f("t1: ", t1);
    //    FW::printVec3f("t2: ", t2);
    //    FW::printVec3f("tin: ", tin);
    //    FW::printVec3f("tout: ", tout);
    //    printf("tstart: %f \n", tstart);
    //    printf("tend: %f \n", tend);
    //    printf(" \n");
    //}

    if (tstart > tend || tend < 0) {
        //No Intersections
        return castresult;
    }

    //Go deeper
    RaycastResult resultL, resultR;
    if(node.left)
        resultL = raycastBvhIterator(orig, dir, reci_dir_unit, *node.left);
    if(node.right)
        resultR = raycastBvhIterator(orig, dir, reci_dir_unit, *node.right);

    return resultL.t < resultR.t ? resultL: resultR;
     
}


} // namespace FW