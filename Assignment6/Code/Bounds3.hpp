//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

// 轴对齐包围盒类
class Bounds3
{
public:
    Vector3f pMin, pMax; // 用于指定包围盒的两个点

    // 默认构造函数，将 pMin 设置为最大值，pMax 设置为最小值
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }

    // 使用一个点构造包围盒，pMin 和 pMax 都设置为该点
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

    // 使用两个点构造包围盒，pMin 和 pMax 分别为两个点的最小值和最大值
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    // 计算对角线向量
    Vector3f Diagonal() const { return pMax - pMin; }

    // 返回最长的轴（0: x轴, 1: y轴, 2: z轴）
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

    // 计算表面积
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    // 计算包围盒的中心点
    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    // 计算两个包围盒的交集
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y), fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y), fmin(pMax.z, b.pMax.z)));
    }

    // 计算点 p 在包围盒中的偏移量
    Vector3f Offset(const Vector3f& p) const
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

    // 判断两个包围盒是否重叠
    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    // 判断点 p 是否在包围盒 b 内
    bool Inside(const Vector3f& p, const Bounds3& b) const
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    // 重载下标运算符，返回 pMin 或 pMax
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    // 判断射线是否与包围盒相交
    inline bool IntersectP(const Ray& ray) const;
};

// 判断射线是否与包围盒相交的实现
inline bool Bounds3::IntersectP(const Ray& ray) const
{
    // invDir: 射线方向的倒数 (1.0/x, 1.0/y, 1.0/z)，因为乘法比除法快
    // dirIsNeg: 射线方向 (x, y, z)，dirIsNeg=[int(x>0), int(y>0), int(z>0)]，用这个来简化逻辑
    // TODO: 测试射线是否与包围盒相交
    // 如果相交，返回 true，否则返回 false

    const Vector3f invDir = ray.direction_inv;
    const std::array<int, 3> dirIsNeg = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    
    if(Inside(ray.origin, *this))
        return true;
        
    float txmin = (pMin.x - ray.origin.x) * invDir.x;
    float txmax = (pMax.x - ray.origin.x) * invDir.x;
    if(txmin > txmax) 
        std::swap(txmin, txmax);
    float tymin = (pMin.y - ray.origin.y) * invDir.y; 
    float tymax = (pMax.y - ray.origin.y) * invDir.y;
    if(tymin > tymax) 
        std::swap(tymin, tymax);
    float tzmin = (pMin.z - ray.origin.z) * invDir.z;
    float tzmax = (pMax.z - ray.origin.z) * invDir.z;
    if(tzmin > tzmax) 
        std::swap(tzmin, tzmax);
    float tmin = std::max(txmin, std::max(tymin, tzmin));
    float tmax = std::min(txmax, std::min(tymax, tzmax));
    if(tmin > tmax)
        return false;

    return true;
}

// 计算两个包围盒的并集
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

// 计算包围盒和点的并集
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H