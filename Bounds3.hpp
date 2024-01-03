//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        // Mine Add:
        // Because the init in the tringle.cpp 101-107 they start at the same and 
        // just be samller or greater than the initValue, I think this is trival
        // in the bunny Object.

        // But it useful for a sigleTariangel.

        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3 &b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f &p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3 &b1, const Bounds3 &b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f &p, const Bounds3 &b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f &operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray &ray, const Vector3f &invDir,
                           const std::array<int, 3> &dirisNeg) const;

    Bounds3 Clip(const Bounds3 &tri, const Bounds3 &Voxel);
};

inline bool Bounds3::IntersectP(const Ray &ray, const Vector3f &invDir,
                                const std::array<int, 3> &dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

    // Mine add : Alhough, there are a lot of AABB BVH algorithm publish in the Network;
    //              such as tavianator.com/2022 which claim a branchless methond.
    //            Whatever for my first AABB, I just want it will be easy and understandable.
    //              and the algorithm in the lecture didn't consider the parallel case.
    //            So I do it with the form teached by Justin Solomon(A Youtuber <Introduction to Computer Graphics 12.>)
    //              also, the idea is same;
    /*
    For each dimension.
        if Rd_x=0 (Ray is parallel) And (Ro_x<X1 or Ro_x<X2) -> NO Intersection

    For each dimension ,calculate intersection distance t1 and t2(which is tenter and texist is not clear now but suppose t1 is enter):
        t1 = (X1 - Ro_x) / Rd_x;
        t2 = (X2 - Ro_x) / Rd_x;
        if(t1 > t2) swap(t1,t2);
        Maintain an interval [t_start, t_end], intersect with current dimention
        if t1>t_start, tstart=t1;
        if t2<t_end, tend=t2;

    if t_start>t_end, box missed;
    if t_end<tmin=0.0, box is behind;
    if t_start>tmin, cloest intersection at t_start; else(t_start<=tmin=0.0) cloest intersection at t_end inffer that Ray.orgion in box;
    */

    if ((ray.direction.x * ray.direction.y * ray.direction.z) == 0)
    {
        if (ray.origin.x < pMin.x || ray.origin.x > pMax.x)
            return false;
        if (ray.origin.y < pMin.y || ray.origin.y > pMax.y)
            return false;
        if (ray.origin.z < pMin.z || ray.origin.z > pMax.z)
            return false;
        return true;
    }

    float t1x = (pMin.x - ray.origin.x) / ray.direction.x;
    float t2x = (pMax.x - ray.origin.x) / ray.direction.x;
    if (t1x > t2x)
        std::swap(t1x, t2x);

    float t1y = (pMin.y - ray.origin.y) / ray.direction.y;
    float t2y = (pMax.y - ray.origin.y) / ray.direction.y;
    if (t1y > t2y)
        std::swap(t1y, t2y);

    float t1z = (pMin.z - ray.origin.z) / ray.direction.z;
    float t2z = (pMax.z - ray.origin.z) / ray.direction.z;
    if (t1z > t2z)
        std::swap(t1z, t2z);

    float t_start = std::max(t1x, std::max(t1y, t1z));
    float t_end = std::min(t2x, std::min(t2y, t2z));

    if(t_start < t_end && t_end>0){
        return true;
    }else{
        return false;
    }

}

inline Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3 &b, const Vector3f &p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

inline Bounds3 Bounds3::Clip(const Bounds3 &tri, const Bounds3 &Voxel){
    Bounds3 ret;
    // first we need to test the two boxes collision or not
    // true : Intersect
    // false: return Voxel
    // test collision Algorithm from developer.mozilla.org  3D collision dectction
    // Intersect Algorithm from the pbr-books.org/3ed-2018 2.6 Bounding Boxes
    // 
    if(tri.pMin.x <= Voxel.pMax.x && tri.pMax.x >= Voxel.pMin.x  
        && tri.pMin.y <= Voxel.pMax.y && tri.pMax.y >= Voxel.pMin.y
        && tri.pMin.z <= Voxel.pMax.z && tri.pMax.z >= Voxel.pMin.z){
            ret.pMin = Vector3f::Max(tri.pMin, Voxel.pMin);
            ret.pMax = Vector3f::Min(tri.pMax, Voxel.pMax);
            return ret;
        }else{
            ret = Voxel;
            return ret;
        }
     
    
}
#endif // RAYTRACING_BOUNDS3_H
