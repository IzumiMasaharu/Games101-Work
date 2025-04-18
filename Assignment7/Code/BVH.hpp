//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct SplitBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(SplitBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    SplitBuildNode* root;

    // BVHAccel Private Methods
    SplitBuildNode* recursiveBuild(std::vector<Object*>objects);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    void getSample(SplitBuildNode* node, float p, Intersection &pos, float &pdf);
    void Sample(Intersection &pos, float &pdf);
};

struct SplitBuildNode {
    Bounds3 bounds;
    SplitBuildNode *left;
    SplitBuildNode *right;
    Object* object;
    float area;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    SplitBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};




#endif //RAYTRACING_BVH_H
