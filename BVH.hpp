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

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct SAHBuildNode;
// add SAHAccel Forward Declarations

// My Added
struct Plane{
    int splitAxis;
    float splitDis;
    enum class PlaneSide {LEFT, RIGHT};
    PlaneSide ps;
    //float cost;

    Plane(){
        splitAxis = 0;
        splitDis = 0;
        ps = PlaneSide::LEFT;
        //cost = 0;
    };

    Plane(int a, float b):splitAxis(a), splitDis(b){ps = PlaneSide::LEFT;};
};

struct EventNode{
    Plane plane;
    int eventType = 0;
    Object* object = nullptr;
};

struct EventsXYZ{
    std::vector<EventNode> x;
    std::vector<EventNode> y;
    std::vector<EventNode> z;

    inline std::vector<EventNode>& operator[](int index) {
        switch(index){
            case 0:
                return x;
                break;
            case 1:
                return y;
                break;
            case 2:
                return z;
                break;
            default:
                return x;
                break;
        }
    }
};

struct NumOfTri{
    int nLeft[4];
    int nPlane[4];
    int nRight[4];
};

struct SplitEAT{
    EventsXYZ distEventLeft;
    EventsXYZ distEventRight;
    std::vector<Object*> objLeft;
    std::vector<Object*> objRight;
};
// end

struct BVHPrimitiveInfo; // no use

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, Bounds3 bounds, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    Intersection getIntersection(SAHBuildNode* node, const Ray& ray)const;// add


    bool IntersectP(const Ray &ray) const;
    BVHBuildNode* root;

    SAHBuildNode* rootSAH;// add

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    SAHBuildNode* recursiveSAHBuild(std::vector<Object*>objects, EventsXYZ Events, Bounds3 bounds, int kdDepth=0);// add

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    // Mine add
    EventsXYZ Events;// add
    bool generateEvents(std::vector<Object*> p);// add
    Plane findPlane(int triNum, Bounds3 bound, EventsXYZ& Events);
    std::pair<float, Plane::PlaneSide> SAH(Bounds3 bounds, Plane p, int Nl, int Nr, int Np);
    SplitEAT ClassifyLeftRightBoth(std::vector<Object*> objs, EventsXYZ& Events, Plane p, std::vector<Bounds3> bounds);
    static bool comp(EventNode f1, EventNode f2){
        if(f1.plane.splitDis < f2.plane.splitDis || (f1.plane.splitDis == f2.plane.splitDis && f1.eventType < f2.eventType)){
                return true;
            }else{
                return false;
            }
        }
};

struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};

struct SAHBuildNode {
    Bounds3 bounds;
    float splitDis;
    SAHBuildNode *left;
    SAHBuildNode *right;
    std::vector<Object*> object;
    bool isleaf;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    SAHBuildNode(){
        bounds = Bounds3();
        float splitDis = 0;
        left = nullptr;right = nullptr;
        //object = nullptr;
        isleaf = false;
    }

    std::vector<Bounds3> getBox(){
        std::vector<Bounds3> b;
        Vector3f bLMax = bounds.pMax;
        //std::cout<<bLMax<<std::endl;
        bLMax[splitAxis] = splitDis;
        //std::cout<<bLMax<<std::endl;
        Bounds3 B1{bounds.pMin, bLMax};
        b.push_back(B1);
        Vector3f bRMin = bounds.pMin;
        bRMin[splitAxis] = splitDis;
        Bounds3 B2{bRMin, bounds.pMax};
        b.push_back(B2);

        return b;
    }
};



#endif //RAYTRACING_BVH_H
