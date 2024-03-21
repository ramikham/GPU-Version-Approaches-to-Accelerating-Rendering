//
// Created by ramik on 2/19/2024.
//

#ifndef UNTITLED_AABB_H
#define UNTITLED_AABB_H

#include "Primitive.h"

class AABB {
public:
    // Constructor
    // -----------------------------------------------------------------------
    __device__ AABB() {}

    __device__ AABB(const Vec3D& min, const Vec3D& max) : minimum(min), maximum(max) {
        bounds[0] = min;
        bounds[1] = max;
    }

    // Overloaded Functions
    // -----------------------------------------------------------------------
    __device__ bool intersection(const Ray& r, float t_min, float t_max) const {
        // Does the ray intersect the AABB?

        double tmin, tmax, tymin, tymax, tzmin, tzmax;

        tmin = (bounds[r.sign[0]].x() - r.get_ray_origin().x()) * r.inv_direction.x();
        tmax = (bounds[1 - r.sign[0]].x() - r.get_ray_origin().x()) * r.inv_direction.x();
        tymin = (bounds[r.sign[1]].y() - r.get_ray_origin().y()) * r.inv_direction.y();
        tymax = (bounds[1 - r.sign[1]].y() - r.get_ray_origin().y()) * r.inv_direction.y();

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        tzmin = (bounds[r.sign[2]].z() - r.get_ray_origin().z()) * r.inv_direction.z();
        tzmax = (bounds[1 - r.sign[2]].z() - r.get_ray_origin().z()) * r.inv_direction.z();

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        return ((tmin < t_max) && (tmax > t_min));
    }

    // Getters
    // -----------------------------------------------------------------------
    __device__ const Vec3D& get_min() const {
        return minimum;
    }

    __device__ const Vec3D& get_max() const {
        return maximum;e
    }
public:
    // Data Members
    // -----------------------------------------------------------------------
    Vec3D minimum;                      // point at the minimum corner of the box
    Vec3D maximum;                      // point at the maximum corner of the box
    Vec3D bounds[2];                    // the two corners of the box
};

// Supporting Functions
// -----------------------------------------------------------------------
__device__ inline AABB enclosing_box(const AABB& box_0, const AABB& box_1) {
    // Constructs a box surrounding two other boxes: box_0 and box_1

    const Vec3D minimum_point{
            fmin(box_0.get_min().x(), box_1.get_min().x()),
            fmin(box_0.get_min().y(), box_1.get_min().y()),
            fmin(box_0.get_min().z(), box_1.get_min().z())
    };

    const Vec3D maximum_point{
            fmax(box_0.get_max().x(), box_1.get_max().x()),
            fmax(box_0.get_max().y(), box_1.get_max().y()),
            fmax(box_0.get_max().z(), box_1.get_max().z())
    };

    return { minimum_point, maximum_point };
}

#endif //UNTITLED_AABB_H

