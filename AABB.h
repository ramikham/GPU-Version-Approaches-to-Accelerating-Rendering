//
// Created by ramik on 2/19/2024.
//

#ifndef UNTITLED_AABB_H
#define UNTITLED_AABB_H

#include "Primitive.h"

class AABB {
public:
    __device__ AABB() {}
    __device__ AABB(const Vec3D& min, const Vec3D& max) : minimum(min), maximum(max) {}

    __device__ const Vec3D& min() const {
        return minimum;
    }

    __device__ const Vec3D& max() const {
        return maximum;
    }

    __device__ bool intersection(const Ray& r, float t_min, float t_max) const {
        /*
        for (int a = 0; a < 3; a++) {
            auto t0 = fmin((minimum[a] - r.get_ray_origin()[a]) / r.get_ray_direction()[a],
                           (maximum[a] - r.get_ray_origin()[a]) / r.get_ray_direction()[a]);
            auto t1 = fmax((minimum[a] - r.get_ray_origin()[a]) / r.get_ray_direction()[a],
                           (maximum[a] - r.get_ray_origin()[a]) / r.get_ray_direction()[a]);
            t_min = fmax(t0, t_min);
            t_max = fmin(t1, t_max);
            if (t_max <= t_min)
                return false;
        }
        return true;*/
        // We need the inverse of the ray's direction. Also, I don't want to divide by the
// inverse; I will instead multiply by the reciprocal.
        Vec3D ray_direction = r.get_ray_direction();
        Vec3D ray_origin = r.get_ray_origin();
        float inv_direction_x = 1.0f / ray_direction.x();
        float inv_direction_y = 1.0f / ray_direction.y();
        float inv_direction_z = 1.0f / ray_direction.z();

        float t0_x = fminf((minimum.x() - ray_origin.x()) * inv_direction_x, (maximum.x() - ray_origin.x()) * inv_direction_x);
        float t1_x = fmaxf((minimum.x() - ray_origin.x()) * inv_direction_x, (maximum.x() - ray_origin.x()) * inv_direction_x);
        t_min = fmaxf(t0_x, t_min);
        t_max = fminf(t1_x, t_max);
        if (t_max <= t_min)
            return false;

        float t0_y = fminf((minimum.y() - ray_origin.y()) * inv_direction_y, (maximum.y() - ray_origin.y()) * inv_direction_y);
        float t1_y = fmaxf((minimum.y() - ray_origin.y()) * inv_direction_y, (maximum.y() - ray_origin.y()) * inv_direction_y);
        t_min = fmaxf(t0_y, t_min);
        t_max = fminf(t1_y, t_max);
        if (t_max <= t_min)
            return false;

        float t0_z = fminf((minimum.z() - ray_origin.z()) * inv_direction_z, (maximum.z() - ray_origin.z()) * inv_direction_z);
        float t1_z = fmaxf((minimum.z() - ray_origin.z()) * inv_direction_z, (maximum.z() - ray_origin.z()) * inv_direction_z);
        t_min = fmaxf(t0_z, t_min);
        t_max = fminf(t1_z, t_max);
        if (t_max <= t_min)
            return false;

        return true;
    }
public:
    Vec3D minimum;
    Vec3D maximum;
};

__device__ inline AABB enclosing_box(const AABB& box, const AABB& other_box) {
    const Vec3D lower{
            fmin(box.min().x(), other_box.min().x()),
            fmin(box.min().y(), other_box.min().y()),
            fmin(box.min().z(), other_box.min().z())
    };

    const Vec3D upper{
            fmax(box.max().x(), other_box.max().x()),
            fmax(box.max().y(), other_box.max().y()),
            fmax(box.max().z(), other_box.max().z())
    };

    return { lower, upper };
}

#endif //UNTITLED_AABB_H
